import os


HOME_DIR = os.getenv("HOME")
CURA_ENGINE_DIR = os.path.join(HOME_DIR, 'CuraEngine/build/CuraEngine')
JSON_FILE_DIR = 'anet.def.json'

command = "{} slice -v -j {}".format(CURA_ENGINE_DIR, JSON_FILE_DIR)
# command = f"{CURA_ENGINE_DIR} slice -v -j {JSON_FILE_DIR}"
print(command)