import sys
import os
import re
import glob





# program start
if __name__ == "__main__":
    
    if len(sys.argv) < 5:
        print('copyImages.py <image_dir> <start-timestamp> <start-index> <number>')
        sys.exit(1)

    image_dir = os.path.abspath(sys.argv[1])
    start_timestamp = int(sys.argv[2])
    start_index = int(sys.argv[3])
    copy_num = int(sys.argv[4])
    
    timestamp_list = []
    
    list_of_files = sorted( filter( os.path.isfile, glob.glob(image_dir +'/depth16-*.png') ) )    
    for file_name in list_of_files:
        match = re.match("depth16-(\\d+).png", os.path.basename(file_name))
        if match == None:
            continue
        print(file_name)
        timestamp = int(match.group(1))
        if timestamp >= start_timestamp:
            timestamp_list.append(timestamp)
        
    ind = start_index    
    for timestamp in timestamp_list:
        if ind >= start_index + copy_num:
            break
        os.system('cp ' + image_dir + '/depth16-' + str(timestamp) + '.png   depImages/depth16-' + str(ind) +'.png')
        os.system('cp ' + image_dir + '/disp16-' + str(timestamp) + '.png   dispImages/disp16-' + str(ind) +'.png')
        os.system('cp ' + image_dir + '/vDisp16-' + str(timestamp) + '.png   vDisp/vDisp16-' + str(ind) +'.png')
        os.system('cp ' + image_dir + '/col-' + str(timestamp) + '.png   colImages/col-' + str(ind) +'.png')
        os.system('cp ' + image_dir + '/depth-' + str(timestamp) + '.png   depthColor/depth-' + str(ind) +'.png')
        ind = ind + 1


    

