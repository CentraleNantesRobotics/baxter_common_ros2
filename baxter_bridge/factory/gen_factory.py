#!/usr/bin/env python3

import os
import yaml
import sys

debug = '-d' in sys.argv

if '-s' in sys.argv:
    # scan topics, make sure to be on a ROS 1 terminal connected to Baxter
    if 'ROS_MASTER_URI' not in os.environ or 'baxter.local' not in os.environ['ROS_MASTER_URI']:
        print('Not connected to Baxter')
        sys.exit(0)

    if os.environ['ROS_DISTRO'] not in ('melodic','noetic'):
        print('Not in a ROS 1 terminal')
        sys.exit(0)

    from subprocess import check_output

    def getlines(cmd):
        return check_output(cmd.split()).decode().splitlines()

    topics = getlines('rostopic list')
    infos = []
    for i,topic in enumerate(topics):
        print(f'Checking {topic} ({i+1}/{len(topics)})')
        infos += getlines(f'rostopic info {topic}')

pkg_dir = os.path.abspath(os.path.dirname(__file__) + '/..')

ignores = ['/tf','/rosout', '/tf_static','/rosout_agg', '/robot/urdf']
deprecated = ['arm_navigation_msgs', 'MotorControlMsgs']

builtin = ['bool','string','byte','float32','float64']
builtin += [f'{i}{s}' for i in ('int','uint') for s in (8,16,32,64)]

MSG_BASE = 0
MSG_BASE_ARRAY = 1
MSG_CUSTOM = 2


def check_msg_type(msg):
    is_array = msg[-1] == ']' and msg[-2] != '['
    if '[' in msg:
        msg = msg.split('[')[0]
    base_type = msg in builtin

    if base_type:
        if is_array:
            return MSG_BASE_ARRAY
        return MSG_BASE
    return MSG_CUSTOM

    #return msg[0].islower() and not is_array and '/' not in msg and any(b in msg for b in builtin)


def ros2_include(msg):
    # ROS 2 uses only lower case
    for src,dst in (('URDF','urdf'),('SEA','sea')):
        msg = msg.replace(src,dst)
    include = ''
    for i,c in enumerate(msg):
        if c.isupper():
            if msg[i-1].islower() or msg[i-1].isdigit():
                include += '_'
            include += c.lower()
        else:
            include += c
    # a few exceptions
    for src,dst in (('/','/msg/'),('uint','u_int'),('iostate','io_state')):
        include = include.replace(src,dst)
    return include+'.hpp'


with open(pkg_dir + '/factory/baxter.yaml') as f:
    infos = yaml.safe_load(f)

# add all cameras which may not be activated
for cam in ('head_camera', 'right_hand_camera', 'left_hand_camera'):

    base_topic = f'/cameras/{cam}/'

    for sub, msg in (('camera_info', 'CameraInfo'),
                     ('camera_info_std', 'CameraInfo'),
                     ('image', 'Image')):
        topic = base_topic + sub
        infos[topic] = {'pub': ['/baxter_cams (http://baxter.local:42344/)'],
                        'type': 'sensor_msgs/' + msg}


def valid_node(node):
    return 'baxter.local' in node # and 'rcloader_' not in node


ros1to2 = set()
ros2to1 = set()
topics = {'publishers': {}, 'subscribers': {}}
for topic, info in infos.items():
    if topic in ignores or any(d in info['type'] for d in deprecated):
        continue
    if debug:
        if not any(keep in info['type'] for keep in ['Image','JointState','JointCommand','CameraInfo']):
            continue

    pubs = [p for p in info['pub'] if valid_node(p)] if 'pub' in info else []
    subs = [s for s in info['sub'] if valid_node(s)] if 'sub' in info else []

    if len(pubs) and len(subs):
        print(f'Loop detected ({topic})')

        #print(' subscribers: \n   ' + '\n   '.join([s for s in info['sub'] if valid_node(s)]))
        #print(' publishers:\n   ' + '\n   '.join([s for s in info['pub'] if valid_node(s)]))
        rt_pub = any('/realtime_loop' in p for p in pubs)
        rt_sub = any('/realtime_loop' in s for s in subs)
        if rt_pub and not rt_sub:
            print('  -> Favoring publishing from /realtime_loop\n')
            subs = []
        elif rt_sub and not rt_pub:
            print('  -> Favoring subscribing from /realtime_loop\n')
            pubs = []
        else:
            print('  -> /realtime_loop is both subscriber and publisher\n')
    if len(subs):
        topics['subscribers'][topic] = info['type']
        ros2to1.add(info['type'])
    if len(pubs):
        topics['publishers'][topic] = info['type']
        ros1to2.add(info['type'])

print()

# special rule for sensor_msgs/CameraInfo
rules = {'ros1_package_name': 'sensor_msgs'}
rules['ros1_message_name'] = 'CameraInfo'
rules['fields_1_to_2'] = {}
for field in 'dkrp':
    rules['fields_1_to_2'][field.upper()] = field

rules = [rules]
for pkg in ('baxter_core_msgs','baxter_maintenance_msgs'):
    with open(pkg_dir + f'/../{pkg}/mapping_rules.yaml') as f:
        rules += yaml.safe_load(f)

# simplify + invert 2 to 1
rules = dict((r['ros1_package_name']+'/'+r['ros1_message_name'],
              dict((v,k) for (k,v) in r['fields_1_to_2'].items())) for r in rules)


def load_message(full_msg):

    pkg,msg = full_msg.split('/')
    if pkg.startswith('baxter_'):
        root = pkg_dir + '/..'
    else:
        root = f'/opt/ros/{os.environ["ROS_DISTRO"]}/share'

    if not os.path.exists(root + f'/{pkg}/msg/{msg}.msg'):
        return None, None

    with open(root + f'/{pkg}/msg/{msg}.msg') as f:
        definition = [line.split('#')[0].strip() for line in f.read().splitlines() if '=' not in line]
    fields = dict(reversed(line.split()[:2]) for line in definition if line)

    for field,sub in fields.items():
        msg_type = check_msg_type(sub)
        if '/' not in sub and msg_type == MSG_CUSTOM:
            fields[field] = f'{pkg}/{sub}'

    rule = dict((f,f) for f in fields)
    if full_msg in rules:
        rule.update(rules[full_msg])
    return fields,rule


def toElem(msg):
    return msg.split('[')[0]


def toROS1(msg):
    return toElem(msg.replace('/', '::'))


def toROS2(msg):
    return toElem(msg.replace('/', '::msg::'))


class Factory:
    def __init__(self, src):
        self.src = src
        self.includes = []
        self.forwards = []

        self.msgs_done = []

        self.tag = '2to1' if '2to1' in src else '1to2'

        self.fact = []

        self.topics = {}

    def add(self, topic, msg):

        if not self.build_fwd(msg):
            return

        self.topics[topic] = msg

        if msg+'.h' in self.includes:
            return

        # ROS 1 uses raw message name as include
        self.includes.append(msg+'.h')
        self.includes.append(ros2_include(msg))

        msg1 = toROS1(msg)
        msg2 = toROS2(msg)

        self.fact.append(f'''if(msg == "{msg}")
  {{
    bridges.push_back(std::make_unique<Bridge_{self.tag}<{msg1}, {msg2}>>
        (topic));
  }}''')

    def build_fwd(self, msg):

        msg = toElem(msg)

        if msg in self.msgs_done:
            return True

        fields, rule = load_message(msg)
        if fields is None:
            print('Cannot load incomplete message ' + msg)
            return False

        to2 = self.tag == '1to2'
        src = toROS2(msg)
        dst = toROS1(msg)

        if to2:
            src,dst = dst,src

        valid = True

        if msg == 'std_msgs/Empty':
            # nothing to do here
            fwd = [f'template<>\nvoid convertMsg(const {src} &, {dst} &)\n{{']

        else:
            fwd = [f'template<>\nvoid convertMsg(const {src} &src, {dst} &dst)\n{{']

        for field2,sub in fields.items():

            if to2:
                src_field = f'src.{rule[field2]}'
                dst_field = f'dst.{field2}'
            else:
                src_field = f'src.{field2}'
                dst_field = f'dst.{rule[field2]}'

            # some special cases
            msg_type = check_msg_type(sub)
            if sub == 'builtin_interfaces/Time':
                if to2:
                    fwd.append(f'  {dst_field} = Bridge::ros2_now();')
                else:
                    fwd.append(f'  {dst_field} = Bridge::ros1_now();')
            elif sub == 'builtin_interfaces/Duration':
                fwd.append(f'  {dst_field}.sec = {src_field}.sec;')
                if to2:
                    fwd.append(f'  {dst_field}.nanosec = {src_field}.nsec;')
                else:
                    fwd.append(f'  {dst_field}.nsec = {src_field}.nanosec;')

            elif msg_type == MSG_BASE_ARRAY or 'bool[' in sub:
                fwd.append(f'  convertMsg({src_field}, {dst_field});')
            elif msg_type == MSG_BASE:
                fwd.append(f'  {dst_field} = {src_field};')
            else:
                valid = valid and self.build_fwd(sub)
                fwd.append(f'  convertMsg({src_field}, {dst_field});')

        self.msgs_done.append(msg)

        if valid:
            print(f'{src} -> {dst}\n')
            fwd.append('}\n')
            self.forwards.append('\n'.join(fwd))
            return True
        print('Cannot load incomplete message ' + msg)
        return False

    def write(self):

        print(f'{self.tag}: {len(self.topics)} topics using {len(self.includes)} messages')

        content = ['//Generated with gen_factory.py, edit is not recommended']
        content += [f'#include <baxter_bridge/bridge_{self.tag}.h>', '#include <baxter_bridge/factory.h>', '//messages']
        for include in self.includes:
            content.append(f'#include <{include}>')
        content.append('\nnamespace baxter_bridge\n{')

        # forwards
        content.append('// converters')
        content += self.forwards

        # topics
        content.append('std::map<std::string, std::string> Factory::topics_' + self.tag + ' = {')
        for topic, msg in self.topics.items():
            content.append(f'  {{"{topic}", "{msg}"}},')
        content[-1] = content[-1][:-1] + '};\n'

        # finish createBridge fct
        content.append(f'void Factory::createBridge_{self.tag}(const std::string &topic, const std::string &msg)')
        content.append('{')
        content.append('  ' + '\n  else '.join(self.fact))
        content.append('}\n}')

        content = '\n'.join(content)

        with open(self.src) as f:
            same = f.read() == content

        if not same:
            print('Updating ' + self.src)
            with open(self.src, 'w') as f:
                f.write(content)
        else:
            print('Not changing ' + self.src)


f12 = Factory(pkg_dir + '/src/factory_1to2.cpp')
f21 = Factory(pkg_dir + '/src/factory_2to1.cpp')

for topic, msg in topics['publishers'].items():
    f12.add(topic, msg)

for topic, msg in topics['subscribers'].items():
    f21.add(topic, msg)


f12.write()
f21.write()
