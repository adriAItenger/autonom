import numpy as np
import math


def getDistance2(ranges,angles):
    global marker_points
    if(len(ranges) > 50):
        p_index,q_index=getFarthestNeighbours(filterInf(ranges))
        p=Point()
        q=Point()
        p.x,p.y=calcPointPos(ranges[p_index],angles[p_index])
        q.x,q.y=calcPointPos(ranges[p_index],angles[q_index])
        r=Point() #a közelebbi pont x kordinátájára kell az távolabbi és az ő y-ának különbségének fele(rajz)
        if ranges[p_index]<ranges[q_index]:
            r.x=p.x
            r.y=abs(q.y-p.y)/2 #nem biztos, hogy pont a fele kell, később adaptívvá is tehetjük(szög és táv függvényében)
        else:
            r.x=q.x
            r.y=abs(p.y-q.y)/2 
            # most erre kellene megkeresni a távot és a kormányszöget
            #azt is meg kell oldani, hogy csak előre nézzen, mikor a pontokat keresi update: farthestneighbours fileban elvileg javitottam
        distance=r.x

        return distance

        '''center1_min_index = np.where(math.radians(160) < angles)[0][0]
        print(np.where(math.radians(160) < angles))
        center1_max_index = np.where(math.radians(179.9) < angles)[0][0]
        tmp1 = np.arange(center1_min_index, center1_max_index, 1)
        center2_min_index = np.where(math.radians(-179.9) < angles)[0][0]
        center2_max_index = np.where(math.radians(-160) < angles)[0][0]
        tmp2 = np.arange(center2_min_index, center2_max_index, 1)
        tmp = np.concatenate((tmp1, tmp2))
        max_x = -10.0'''
        for t in tmp:
            point = Point()
            point.x, point.y = calcPointPos(ranges[t], angles[t])
            if not math.isinf(point.x):
                # find max (flipped upside min)
                if point.x > max_x:
                    max_x = point.x
            """
            # debug
            if not math.isinf(point.x):
                point.z = 1
                marker_points.points.append(point)
            """
        if math.isinf(max_x):
            max_x = -5.0
        # within 40 cm reverse - tolatas    
        if max_x > -0.4:
            max_x = 0.5
            #print("tolatas - backward motion")            
        """
        # debug
        marker_points.header.frame_id = "laser"
        """
        distance = max_x
    #else: 
        distance = 0.4
    #return distance


a=np.arange(20,25,1)
print(a)
