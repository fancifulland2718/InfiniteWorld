from nav import main

# task json path
json_path ="you should put the path of the json file here"
# process image index
img_index = [0]

# run
state, path_len, NE = main(json_path,img_index=img_index)

# print result
if state:
    print("state:success")  
else:
    print("state:Fail")