from Mesh_pb2 import Points

path_to_proto = "/Users/occamlab/Documents/DepthData/depth_benchmarking/HccdFYqmqETaJltQbAe19bnyk2e2/F231CF41-FBCE-4C5B-8E88-BD05FEDB7591/0005/pointcloud.pb"

with open(path_to_proto, 'rb') as f:
    cloud = Points()
    cloud.ParseFromString(f.read())

for p, c in zip(cloud.points, cloud.confidences):
    if c == 2:
        print(p.u, p.v, p.w, p.d)
