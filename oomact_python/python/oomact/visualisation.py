import rospy



#TODO
class PoseAnimation3D:
    def __init__(self, frames, data, **args): #probably should take something like a pose generating function
        #TODO
        self.framedata={}
        for f in frames:
            self.framedata[f]=[] #I think this should contain poses


    def datacallback(self, data):
        #datatype should be posestampedwithsensorname
        
        framedata[framename].append(framepose)

    def animate(self, position, pose, **kwargs):
        #This should do an animation of a frame
        #think about how to determine framenr
        for frame, pose in framedata.items():
            if len(pose)==0:
                continue
            #do actuall drawing/moving
            

    def plottrajectory(self, position, **kwargs):
        #This should plot the trajectory in 3D space
        #probably we need to somehow preprocess data
        data=position
        self.ax.plot(position[0,:], position[1,:], position[2,:], **kwargs)

