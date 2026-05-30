import os
import re

def fix_file(filepath):
    with open(filepath, 'r') as f:
        lines = f.readlines()

    new_lines = []
    for line in lines:
        if line.endswith(' \n'):
            line = line.rstrip() + '\n'
        new_lines.append(line)

    with open(filepath, 'w') as f:
        f.writelines(new_lines)

for root, _, files in os.walk('src'):
    for file in files:
        if file.endswith('.py'):
            fix_file(os.path.join(root, file))
