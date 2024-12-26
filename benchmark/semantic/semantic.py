import csv
import json


def read_csv(file_path):
    """
    read the CSV file，return a list of dic
    :param file_path: the csv file path
    :return a list of dic
    """
    with open(file_path, mode='r', encoding='utf-8') as file:
        reader = csv.DictReader(file)
        return list(reader)


class SemanticMake:
    # class of semantic tool
    def __init__(self, scene, semantic):
        with open(scene, 'r') as file:
            data = json.load(file)
        self.obj_list = data['object_instances']
        self.semantic_csv = read_csv(semantic)

        self.obj_semantic = self.process_data()
        
    def process_data(self):
        """
        Process data to rearrange obj semantics based on its room
        :param self.obj_list: a list contain obj dic
        :param self.semantic_csv: semantic date from csv file
        :return: obj semantics by room
        """
        obj_semantic = {}

        for item in self.obj_list:
            template_name = item.get('template_name')

            for row in self.semantic_csv:
                if row['id'] == template_name:
                    find_in = row['foundIn']    # room row

                    if find_in == '':
                        find_in = 'Anywhere'
                    name = row['name'] if row['name'] != '' else 'Anything'

                    if find_in not in obj_semantic:
                        obj_semantic[find_in] = []
    
                    # obj dic
                    obj = {
                        "id": row['id'],
                        "name": name,
                        "room": find_in,
                        "pos": item.get('translation'),
                        "rot": item.get('rotation'),
                        "motion_type": item.get('motion_type'),
                    }

                    obj_semantic[find_in].append(obj)
                    break
    
        return obj_semantic

    def return_semantic_all(self):
        """
        Directly return the complete semantics after sorting
        :return: obj semantics by room
        """
        return self.obj_semantic

    def return_rooms(self):
        """
        Returns the rooms contained in the scene
        :return: rooms contained in the scene
        """
        return [room for room in self.obj_semantic]

    def return_rooms_w_objs(self):
        """
        Returns all rooms and their objs
        :return: all rooms and their objs
        """
        room_w_obj = {}
        for room in self.obj_semantic:
            room_w_obj[room] = [obj['name'] for obj in self.obj_semantic[room]]
        return room_w_obj

    def return_objs_in_room(self, room):
        """
        Returns all objs in the specified room
        :param room: specified room
        :return: all objs in the specified room
        """
        if room in self.obj_semantic:
            return [obj['name'] for obj in self.obj_semantic[room]]
        else:
            print("No %s in the scene" % room)
            return 0

    def find_obj_by_id(self, id):
        """
        Find an obj by id
        :param id: obj id
        :return: information about the specified item (dictionary)
        """
        for room in self.obj_semantic:
            for obj in self.obj_semantic[room]:
                if obj['id'] == id:
                    return obj
        print(print("No %s in the scene" % id))
        return 0

    def find_obj_by_name(self, name):
        """
        Find the obj by name
        there may be multiple objs with the same name
        :param name: obj name
        :return: list of information about the specified obj (dictionary)
        """
        objs = []
        for room in self.obj_semantic:
            for obj in self.obj_semantic[room]:
                if obj['name'] == name:
                    objs.append(obj)
        if not objs:
            print("No %s in the scene" % name)
            return 0
        else:
            return objs

    def find_obj_in_room(self, room, name):
        """
        Find the obj by name and room
        there may be multiple objs with the same name in a room
        :param name: obj name
        :param name: room name
        :return: list of information about the specified obj (dictionary)
        """
        if room not in self.obj_semantic:
            print("No %s in the scene" % room)
            return 0
        objs = []
        for obj in self.obj_semantic[room]:
            if obj['name'] == name:
                objs.append(obj)
        if not objs:
            print("No %s in %s" % (name, room))
            return 0
        else:
            return objs


# json_file_path = './scenes-uncluttered/102343992.scene_instance.json'
# csv_file_path = './semantics/objects.csv'

# s = SemanticMake(json_file_path, csv_file_path)
# print(s.return_semantic_all())
# print(s.return_rooms())
# print(s.return_rooms_w_objs())
# print(s.return_objs_in_room('kitchen'))
# print(s.find_obj_by_id('1efdc3d37dfab1eb9f99117bb84c59003d684811'))
# print(s.find_obj_by_name("60'' TV"))
