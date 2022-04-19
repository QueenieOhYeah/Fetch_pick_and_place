import pickle
import geometry_msgs.msg


def pickling(d: dict):
    filename = "fetch_position"
    outfile = open(filename, 'wb')
    pickle.dump(d, outfile)
    outfile.close()
    

  
def unpickling():
    filename = "fetch_position"
    infile = open(filename, 'rb')
    obj = pickle.load(infile)
    infile.close()
    return obj
    
msg = geometry_msgs.msg.PoseStamped()   
dogs_dict = { 'Ozzy': 3, 'Filou': 8, 'Luna': 5, 'Skippy': 10, 'Barco': 12, 'Balou': 9, 'Laika': 16 }
pickling(msg)
new_dict = unpickling()
print(new_dict)
