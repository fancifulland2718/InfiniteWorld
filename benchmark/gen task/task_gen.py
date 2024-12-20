import random
import json
import os
from semantic.semantic import SemanticMake
from gpt import gpt3_5, gpt4
import pandas as pd


def gen_data(csv_file_path, scene_files):
    """
    generate the ovmm task
    :param csv_file_path: the path of the obj semantic csv file path
    :param scene_files: the id of the scene id
    """
    # Semantic objs that need to be excluded
    useless = ["Anything"]

    # sample_scene = random.sample(scene_files, 1)
    sample_scene = [scene_files + '.scene_instance.json']

    json_file_path = "./scenes-uncluttered/" + sample_scene[0]

    s = SemanticMake(json_file_path, csv_file_path)
    scene_all = s.return_rooms_w_objs()

    input_scene = {}
    for room in scene_all:
        # Semantic rooms that need to be excluded
        if room == 'Anywhere':
            continue
        o = list(set(scene_all[room]) - set(useless))
        if o:
            input_scene[room] = o

    with open("prompt/system.txt", "r") as f:
        prompt_system = f.read()

    with open("prompt/rule.txt", "r") as f:
        prompt_rule = f.read()

    with open("prompt/example.txt", "r") as f:
        prompt_example = f.read()

    prompt_input = "Please observe the above rules strictly. Think step by step.\nINPUT:\n```\nscene: " + json.dumps(input_scene) + "\n```\nOUTPUT:"

    prompt = prompt_rule + "\n" + prompt_example + "\n" + prompt_input
    print(prompt_system)
    print(prompt)

    task = gpt4(prompt_system, prompt)
    if '```' in task:
        task = task[3:-3]
    if "python" in task:
        task = task[6:]
    print(task)
    task_dic = json.loads(task)
    task_dic["Scene"] = sample_scene[0].split('.')[0]
    print(task_dic)
    wrong = False
    objs = []
    for task in task_dic["Subtask list"]:
        if "Move_to" in task:
            obj_room = task[9:-2]
            obj = obj_room.split("_")
            semantic = s.find_obj_in_room(obj[1], obj[0])
            if semantic:
                objs.append([semantic[0]['id'], semantic[0]['name'], semantic[0]['room']])
            else:
                wrong = True
                break
    if wrong:
        return
    task_dic['Target'] = objs
    if task_dic['Task instruction'].endswith('.') or task_dic['Task instruction'].endswith('?'):
        task_dic['Task instruction'] = task_dic['Task instruction'][:- 1]
    file = task_dic['Task instruction']
    if not os.path.isdir('task/' + scene_files + '/' + file):
        os.mkdir('task/' + scene_files + '/' + file)
    with open('task/' + scene_files + '/' + file + '/config.json', 'w') as json_file:
        json.dump(task_dic, json_file, indent=4)
    print("saved")

# scene: the id of the hssd scene
scene = '102343992'

if not os.path.isdir("./task/" + scene):
    os.mkdir("./task/" + scene)

csv_file_path = './semantics/objects.csv'
# scene_files = os.listdir("./scenes-uncluttered")
#
for i in range(10):
    gen_data(csv_file_path, scene)

task_list = os.listdir("./task/" + scene)
print(len(task_list))
