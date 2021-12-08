#!/usr/bin/env python

import os
import yaml
import sys


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

ignores = ['/tf','/rosout']

def is_builtin(msg):    
    return msg[0].islower() and '/' not in msg and any(b in msg for b in ['bool','float','int','string','byte'])

def ros2_include(msg):
    # ROS 2 uses only lower case
    for src,dst in (('URDF','urdf'),('SEA','sea')):    
        msg = msg.replace(src,dst)
    include = ''
    for i,c in enumerate(msg):
        if c.isupper():                
            if msg[i-1].islower() or msg[i-1].isdigit(): include += '_'
            include += c.lower() 
        else:
            include += c
    # a few exceptions
    for src,dst in (('/','/msg/'),('uint','u_int'),('iostates','io_states')):
        include = include.replace(src,dst)
    return include+'.hpp'
    

with open(pkg_dir + '/config/baxter.yaml') as f:
    infos = yaml.safe_load(f)
    
deprecated = ['arm_navigation_msgs']
    
topics = {'publishers': {}, 'subscribers': {}}
for topic, info in infos.items(): 
    if any(d in info['Type'] for d in deprecated): continue
    #if not any(keep in info['Type'] for keep in ['Image','JointState','JointCommand']): continue

    if 'Publishers' in info and any(p for p in info['Publishers'] if 'baxter.local' in p): 
        topics['publishers'][topic] = info['Type'] 
    if 'Subscribers' in info and any(s for s in info['Subscribers'] if 'baxter.local' in s): 
        topics['subscribers'][topic] = info['Type']
    
rules = []
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
        root = '/opt/ros/foxy/share'
    if not os.path.exists(root + f'/{pkg}/msg/{msg}.msg'):
        return None, None
    with open(root + f'/{pkg}/msg/{msg}.msg') as f:        
        definition = [line.split('#')[0].strip() for line in f.read().splitlines() if '=' not in line]
    fields = dict(reversed(line.split()[:2]) for line in definition if line)
    
    for field,sub in fields.items():
        if '/' not in sub and not is_builtin(sub):
            fields[field] = f'{pkg}/{sub}'    
    
    if full_msg in rules:
        return fields, rules[full_msg]
    else:
        return fields, dict((f,f) for f in fields)
    
def toElem(msg):
    return msg.replace('[','').replace(']','')
    
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
        
        # remove strange messages / topics
        if msg[0].isupper(): return
        if any(topic.startswith(i) for i in ignores): return
    
        self.topics[topic] = msg
        
        if msg+'.h' in self.includes: return
            
        # ROS 1 uses raw message name as include
        self.includes.append(msg+'.h')
        self.includes.append(ros2_include(msg))

        msg1 = toROS1(msg)
        msg2 = toROS2(msg)
        
        self.build_fwd(msg)
        
        self.fact.append(f'''if(msg == "{msg}")
  {{
    bridges.push_back(std::make_unique<Bridge_{self.tag}<{msg1}, {msg2}>>
        (topic));
  }}''')    
    
    def build_fwd(self, msg):
        
        msg = toElem(msg)
                
        if msg in self.msgs_done: return True
        
        fields, rule = load_message(msg)
        if fields is None: return True
        
        if msg == 'sensor_msgs/CameraInfo':
            for k in rule:
                if len(k) == 1:
                    rule[k] = k.upper()    
        
        fct = 'convertROS' + self.tag[-1] 
        pkg = msg[:msg.find('/')]
        msg1 = toROS1(msg)
        msg2 = toROS2(msg)
        
        valid = True
        
        to2 = self.tag == '1to2'
        
        if to2:
            fwd = [f'template<>\nauto {fct}(const {msg1} &msg1)']
            fwd += ['{',f'  {msg2} msg2;']
        else:
            src,dst = 'msg2','msg1'
            fwd = [f'template<>\nauto {fct}(const {msg2} &msg2)']
            fwd += ['{',f'  {msg1} msg1;']
            
        if msg == 'std_msgs/Empty':
            fwd[0] = fwd[0][:-5]+')'
                        
        for field2,sub in fields.items():            
            
            if to2:
                src_field = f'msg1.{rule[field2]}'
                dst_field = f'msg2.{field2}'
            else:
                src_field = f'msg2.{field2}'
                dst_field = f'msg1.{rule[field2]}'
            
            # some special cases
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
            elif sub[-1] == ']' and sub[-2] != '[':
                # array
                if is_builtin(sub):
                    fwd.append(f'  std::copy({src_field}.begin(), {src_field}.end(), {dst_field}.begin());')
                else:
                    valid = valid and self.build_fwd(sub)
                    sub = toROS1(sub) if to2 else toROS2(sub)
                    fwd.append(f'  std::transform({src_field}.begin(), {src_field}.end(),')
                    fwd.append(f'                 {dst_field}.begin(),')
                    fwd.append(f'                 {fct}<{sub}>);')                
            elif sub == 'bool[]':
                fwd.append(f'''  std::transform({src_field}.begin(), {src_field}.end(), 
    std::back_inserter({dst_field}), [](auto b){{return static_cast<bool>(b);}});''')
            elif is_builtin(sub):
                fwd.append(f'  {dst_field} = {src_field};')
            else:
                # retrieve full message
                valid = valid and self.build_fwd(sub)
                if '[' in sub:
                    fwd.append(f'  std::transform({src_field}.begin(), {src_field}.end(),')
                    fwd.append(f'                 std::back_inserter({dst_field}),')
                    sub = toROS1(sub) if to2 else toROS2(sub)
                    fwd.append(f'                 {fct}<{sub}>);')
                else:
                    fwd.append(f'  {dst_field} = {fct}({src_field});')
            
        if valid:
            print(f'Generating {fct}<{msg1}>\n')
            
            
            fwd.append(f'  return msg{self.tag[-1]};')
            fwd.append('}\n')
            self.forwards.append('\n'.join(fwd))
            self.msgs_done.append(msg)
            return True
        
        return False       
        
    def write(self):
        
        print(f'{self.tag}: {len(self.topics)} topics using {len(self.includes)} messages')
        
        content = ['//Generated from baxter_io.yaml, edit is not recommended']
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
