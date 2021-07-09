import json
import sys
import os
from distutils import dir_util
import subprocess

SHADER_COMPILER = 'C:/VulkanSDK/1.2.176.1/Bin/glslc.exe'
SETTINGS_FILE = 'assets/config/vk_config.json'

build_type = 'Debug'
if len(sys.argv) > 1:
    build_type = sys.argv[1]

with open(SETTINGS_FILE) as f:
    settings = json.load(f)

print(json.dumps(settings['shaders'], indent=1))

os.chdir('build')

ass_path = settings['asset_path']
for mk, mv in settings['shaders'].items():
    for k, v in mv.items():
        src = ass_path + '/' + v + '.' + k
        out = ass_path + '/' + v + '_' + k + '.spv'
        subprocess.run(
            [SHADER_COMPILER, src.replace('/spv', ''), '-o', out])

# dir_util.copy_tree('../assets', 'assets')

# exit(0)
subprocess.run(['cmake', '..'])
subprocess.run(['cmake', '--build', '.', '--config', build_type])
