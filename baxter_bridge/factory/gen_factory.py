#!/usr/bin/env python3

import os
import yaml

pkg_dir = os.path.abspath(os.path.dirname(__file__) + '/..')

ignores = ['/tf','/rosout', '/tf_static','/rosout_agg', '/robot/urdf']
deprecated = ['arm_navigation_msgs', 'MotorControlMsgs', 'actionlib_msgs']

builtin = ['bool','string','byte','float32','float64']
builtin += [f'{i}{s}' for i in ('int','uint') for s in (8,16,32,64)]

MSG_BASE = 0
MSG_BASE_ARRAY = 1
MSG_CUSTOM = 2


def check_msg_type(msg: str):
    is_array = msg[-1] == ']' and msg[-2] != '['
    if '[' in msg:
        msg = msg.split('[')[0]
    base_type = msg in builtin
    if base_type:
        if is_array:
            return MSG_BASE_ARRAY
        return MSG_BASE
    return MSG_CUSTOM


def ros2_header(msg: str):
    # adapt to snake_case
    def should_prefix(p,n):
        if p.isdigit():
            return True
        return p.isalpha() and (p.islower() or n.islower())

    include = msg[0]
    for i,c in enumerate(msg[1:]):
        if c.isupper() and should_prefix(msg[i], msg[min(i+2, len(msg)-1)]):
            include += '_'
        include += c.lower()

    return include.replace('/','/msg/')+'.hpp'


with open(pkg_dir + '/factory/baxter.yaml') as f:
    infos = yaml.safe_load(f)


def valid_node(node: str) -> bool:
    return 'baxter.local' in node


ros1to2 = set()
ros2to1 = set()
topics = {'publishers': {}, 'subscribers': {}}
for topic, info in infos.items():
    if topic in ignores or any(d in info['type'] for d in deprecated):
        continue

    pubs = [p for p in info['pub'] if valid_node(p)] if 'pub' in info else []
    subs = [s for s in info['sub'] if valid_node(s)] if 'sub' in info else []

    if len(pubs) and len(subs):
        print(f'Loop detected ({topic})')
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
    with open(f'{pkg_dir}/../{pkg}/mapping_rules.yaml') as f:
        rules += yaml.safe_load(f)

# simplify + invert 2 to 1
# only messages for now, only service is hand-coded
rules = dict((r['ros1_package_name']+'/'+r['ros1_message_name'],
              dict((v,k) for (k,v) in r['fields_1_to_2'].items())) for r in rules if 'ros1_message_name' in r and 'fields_1_to_2' in r)


def load_message(full_msg: str):

    pkg,msg = full_msg.split('/')
    if pkg.startswith('baxter_'):
        root = pkg_dir + '/..'
    else:
        root = f'/opt/ros/{os.environ["ROS_DISTRO"]}/share'

    msg_path = f'{root}/{pkg}/msg/{msg}.msg'

    if not os.path.exists(msg_path):
        return None, None

    with open(msg_path) as f:
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

        self.direction = '2to1' if '2to1' in src else '1to2'

        self.fact = {}

        self.topics = {}

    def add(self, topic, msg):

        if not self.build_fwd(msg):
            return

        if topic is not None:
            self.topics[topic] = msg

        if msg+'.h' not in self.includes:
            # ROS 1 uses raw message name as include
            self.includes.append(msg+'.h')
            self.includes.append(ros2_header(msg))

        msg1 = toROS1(msg)
        msg2 = toROS2(msg)

        if msg1 not in self.fact:
            self.fact[msg1] = f'''if(msg == "{msg}")
    bridges.push_back(std::make_unique<Bridge_{self.direction}<{msg1}, {msg2}>>(topic));'''

    def build_fwd(self, msg):

        msg = toElem(msg)

        if msg in self.msgs_done:
            return True

        fields, rule = load_message(msg)
        if fields is None:
            print('Cannot load incomplete message ' + msg)
            return False

        to2 = self.direction == '1to2'
        src = toROS2(msg)
        dst = toROS1(msg)

        if to2:
            src,dst = dst,src

        valid = True

        if msg == 'std_msgs/Empty':
            # nothing to do here
            fwd = [f'template<>\nvoid convert(const {src} &, {dst} &)\n{{']
        else:
            fwd = [f'template<>\nvoid convert(const {src} &src, {dst} &dst)\n{{']

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
            elif msg == 'baxter_core_msgs/JointCommand' and field2 == 'mode':
                # use safe flag to ensure position or velocity only
                fwd.append('#ifdef BAXTER_BRIDGE_SAFE_CMD')
                fwd.append(f'  convert(std::min({src_field},2), {dst_field});')
                fwd.append('#else')
                fwd.append(f'  convert({src_field}, {dst_field});')
                fwd.append('#endif')
            elif msg_type in (MSG_BASE, MSG_BASE_ARRAY) or 'bool[' in sub or 'byte[' in sub:
                fwd.append(f'  convert({src_field}, {dst_field});')
            elif msg_type == MSG_BASE:
                fwd.append(f'  {dst_field} = {src_field};')
            else:
                valid = valid and self.build_fwd(sub)
                fwd.append(f'  convert({src_field}, {dst_field});')

        self.msgs_done.append(msg)

        if valid:
            print(f'{src} -> {dst}\n')
            fwd.append('}\n')
            self.forwards.append('\n'.join(fwd))
            return True
        #print('Cannot load incomplete message ' + msg)
        return False

    def write(self, src):

        print(f'{self.direction}: {len(self.topics)} topics using {len(self.includes)} messages')

        content = ['//Generated with gen_factory.py, edit is not recommended']
        content += [f'#include <baxter_bridge/bridge_{self.direction}.h>', '#include <baxter_bridge/factory.h>', '//messages']
        for include in self.includes:
            content.append(f'#include <{include}>')
        content.append('\nnamespace baxter_bridge\n{')

        # forwards
        content.append('// converters')
        content += self.forwards

        if src is None:
            print(content)
            return

        # topics
        content.append('std::map<std::string, std::string> Factory::topics_' + self.direction + ' = {')
        for topic, msg in self.topics.items():
            content.append(f'  {{"{topic}", "{msg}"}},')
        content[-1] = content[-1][:-1] + '};\n'

        # finish createBridge fct
        content.append(f'void Factory::createBridge_{self.direction}(const std::string &topic, const std::string &msg)')
        content.append('{')
        content.append('  ' + '\n  else '.join(self.fact.values()))
        content.append('}\n}')

        content = '\n'.join(content)

        with open(src) as f:
            same = f.read() == content

        if not same:
            print('Updating ' + src)
            with open(src, 'w') as f:
                f.write(content)
        else:
            print('Not changing ' + src)


f12 = Factory('1to2')
f21 = Factory('2to1')


# add all cameras which may not be activated
for cam in ('head', 'right_hand', 'left_hand'):
    base_topic = f'/cameras/{cam}_camera/'
    for sub, msg in (('camera_info', 'CameraInfo'),
                     ('camera_info_std', 'CameraInfo'),
                     ('image', 'Image')):
        f12.add(base_topic + sub, 'sensor_msgs/' + msg)

# if we want to forward a particular tf message - not included in ROS 1 baxter_legacy
# f12.add('/tf_manual', 'tf2_msgs/TFMessage')

# messages used for IK bridge, no attached topic
f21.add(None, 'sensor_msgs/JointState')
f21.add(None, 'geometry_msgs/PoseStamped')


for topic, msg in topics['publishers'].items():
    f12.add(topic, msg)

# we might want to forward any classical message from ROS 1 to 2
# used to have all ROS 2 bridge receive / forward a global topic
msg_root = f'/opt/ros/{os.environ["ROS_DISTRO"]}/share'
for pkg in ('sensor_msgs', 'std_msgs', 'geometry_msgs'):
    msg_pkg = f'{msg_root}/{pkg}/msg'
    for msg in os.listdir(msg_pkg):
        if msg.endswith('.msg'):
            f12.add(None, f'{pkg}/{msg[:-4]}')

for topic, msg in topics['subscribers'].items():
    f21.add(topic, msg)


f12.write(pkg_dir + '/src/factory_1to2.cpp')
f21.write(pkg_dir + '/src/factory_2to1.cpp')
