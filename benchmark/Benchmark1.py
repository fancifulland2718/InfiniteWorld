from nav import main

# task json path
json_path ="/data1/lfwj/npc/selected_task2/104862501_172226556/2/config.json"
# process image index
img_index = [0]

# run
state, path_len, NE = main(json_path,img_index=img_index)

# print result
if state:
    print("state:success")  
else:
    print("state:Fail")