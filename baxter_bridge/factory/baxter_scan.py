#!/usr/bin/env python3

from subprocess import check_output
import yaml

def run(cmd):
    return check_output(cmd.split()).decode('utf-8')
        
topics = run('rostopic list').splitlines()

infos = {}
cnt = 0
for topic in topics:
    cnt+=1
    print(f'Scanning {cnt} / {len(topics)}')
    infos[topic] = run(f'rostopic info {topic}')
    

for topic in infos:
    lines = infos[topic].splitlines()
    step = ''
    topic_info = {}
    for line in lines:
        if line == '':
            continue
        if line.startswith('Type'):
            topic_info['type'] = line[6:]
            continue
        if line.startswith('Publishers'):
            step = 'pub'
            continue
        if line.startswith('Subscribers'):
            step = 'sub'
            continue
        if '*' in line:
            node = line.split(' * ')[1]
            if step not in topic_info:
                topic_info[step] = []
            topic_info[step].append(node) 
        infos[topic] = topic_info

with open('baxter.yaml','w') as f:
    yaml.safe_dump(infos, f)
