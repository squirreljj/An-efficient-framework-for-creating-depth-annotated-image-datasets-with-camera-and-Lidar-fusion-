from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import struct
import datetime
import yaml
import os
# create reader instance and open for reading
with open("../start.yaml", "r") as file:
    data = yaml.load(file, Loader=yaml.Loader)
pc_reader_input_path=data["pc_reader_input_path"]
pc_reader_output_path=data["pc_reader_output_path"]
os.mkdir(pc_reader_output_path)
with Reader(pc_reader_input_path) as reader:
    # topic and msgtype information is available on .connections list
    for connection in reader.connections:
        print(connection.topic, connection.msgtype)
    # iterate over messages
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/rslidar_points':
            msg = deserialize_cdr(rawdata, connection.msgtype)
            field_offsets = {field.name: field.offset for field in msg.fields}
            #print(msg.fields)
            points = []
            #print(msg.header.stamp.sec)
            #print(msg.header.stamp.nanosec)
            for i in range(0, len(msg.data), msg.point_step):
                # extract the data for each field using the field_offsets dictionary
                x = msg.data[i+field_offsets['x']: i+field_offsets['x']+4]
                y = msg.data[i+field_offsets['y']: i+field_offsets['y']+4]
                z = msg.data[i+field_offsets['z']: i+field_offsets['z']+4]
                intensity = msg.data[i+field_offsets['intensity']: i+field_offsets['intensity']+4]
                ring = msg.data[i+field_offsets['ring']: i+field_offsets['ring']+2]
                timestamp = msg.data[i+field_offsets['timestamp']: i+field_offsets['timestamp']+8]
                # convert the data from bytes to the expected type
                x = struct.unpack('f', x)[0]
                y = struct.unpack('f', y)[0]
                z = struct.unpack('f', z)[0]
                intensity = struct.unpack('f', intensity)[0]
                ring = struct.unpack('H', ring)[0]
                timestamp = struct.unpack('d', timestamp)[0]
                points.append((x, y, z, intensity, ring,timestamp))
                #print('x:'+str(x))
                #print('y:'+str(y))  
                #print('z:'+str(z))
                #print('intensity:'+str(intensity))
                #print('ring:'+str(ring))
                #print('timestamp:'+str(timestamp))
                timestamp_seconds = timestamp / 1e9
                #dt = datetime.datetime.fromtimestamp(timestamp_seconds)
                #print(dt)
            # write the data as a PCD file
            with open(pc_reader_output_path+str(msg.header.stamp.sec)+'.'+str(msg.header.stamp.nanosec).zfill(9)+'.pcd', 'w') as f:
                f.write('# .PCD v.7 - Point Cloud Data file format\n')
                f.write('VERSION .7\n')
                f.write('FIELDS x y z intensity ring timestamp\n')
                f.write('SIZE 4 4 4 4 2 8\n')
                f.write('TYPE F F F F U D\n')
                f.write('COUNT 1 1 1 1 1 1\n')
                f.write('WIDTH {}\n'.format(len(points)))
                f.write('HEIGHT 1\n')
                f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
                f.write('POINTS {}\n'.format(len(points)))
                f.write('DATA ascii\n')
                for point in points:
                    f.write('{:.6f} {:.6f} {:.6f} {:.6f} {:d} {:f}\n'.format(*point))
            

