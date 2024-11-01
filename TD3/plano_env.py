import math
import os
import random
import subprocess
import time
from os import path

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from squaternion import Quaternion
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import csv

#GOAL_REACHED_DIST = 0.4
GOAL_REACHED_DIST = 0.35
COLLISION_DIST = 0.158
TIME_DELTA = 0.1


# Check if the random goal position is located on an obstacle and do not accept it if it is
def check_pos(x, y):
    goal_ok = True
    """
    if -3.8 > x > -6.2 and 6.2 > y > 3.8:
        goal_ok = False

    if -1.3 > x > -2.7 and 4.7 > y > -0.2:
        goal_ok = False

    if -0.3 > x > -4.2 and 2.7 > y > 1.3:
        goal_ok = False

    if -0.8 > x > -4.2 and -2.3 > y > -4.2:
        goal_ok = False

    if -1.3 > x > -3.7 and -0.8 > y > -2.7:
        goal_ok = False

    if 4.2 > x > 0.8 and -1.8 > y > -3.2:
        goal_ok = False

    if 4 > x > 2.5 and 0.7 > y > -3.2:
        goal_ok = False

    if 6.2 > x > 3.8 and -3.3 > y > -4.2:
        goal_ok = False

    if 4.2 > x > 1.3 and 3.7 > y > 1.5:
        goal_ok = False

    if -3.0 > x > -7.2 and 0.5 > y > -1.5:
        goal_ok = False

    if x > 4.5 or x < -4.5 or y > 4.5 or y < -4.5:
        goal_ok = False
    """
    return goal_ok
"""
goal_list1 =[[0.252370598591549, 0.256482857142857],
 [0.403792957746479, 3.15922525],
 [0.959008274647887, 4.72402828571429],
 [0.403792957746479, 5.07948203571429],
 [-1.36280123239437, 5.29138953571429],
 [-3.48271426056338, 6.66242542857143],
 [-6.35973908450704, 7.38891507142857],
 [-8.47965211267606, 8.96306739285714],
 [-7.06637676056338, 10.08020775],
 [-6.05689436619718, 11.6035809642857],
 [-5.40073080985916, 12.92383775],
 [-5.35025669014085, 14.0917572142857],
 [-7.11685088028169, 15.3104557857143],
 [-5.55215316901408, 16.8846081071429],
 [-4.03792957746479, 19.0681097142857],
 [-3.12939542253521, 21.2516113214286],
 [-1.21137887323944, 23.0796591785714]]

goal_list2 =[[0.252370598591549, 0.506232857142857],
 [0.10094823943662, 0.709349285714286],
 [-0.908534154929578, 1.521815],
 [-1.11043063380282, 2.13116428571429],
 [-0.858060035211268, 2.79129267857143],
 [-0.10094823943662, 3.19752553571429],
 [-0.201896478873239, 3.6545375],
 [-1.0094823943662, 3.85765392857143],
 [-1.56469771126761, 4.21310767857143],
 [-2.01896478873239, 4.41622410714286],
 [-2.67512834507042, 5.12713160714286],
 [-2.0694389084507, 5.0763525],
 [-2.11991302816901, 5.53336446428571],
 [-2.22086126760563, 6.24427196428571],
 [-2.67512834507042, 6.65050482142857],
 [-3.5836625, 6.95517946428571],
 [-4.34077429577465, 7.9199825],
 [-5.95594612676056, 8.68166910714286],
 [-7.26827323943662, 9.54491392857143],
 [-8.12633327464789, 10.3066005357143],
 [-9.03486742957747, 10.7636125],
 [-7.97491091549296, 10.9667289285714],
 [-7.6720661971831, 11.6776364285714],
 [-6.76353204225352, 12.6424394642857],
 [-5.90547200704225, 13.3533469642857],
 [-5.35025669014085, 13.3025678571429],
 [-5.24930845070423, 14.2165917857143],
 [-5.0978860915493, 15.2321739285714],
 [-5.0978860915493, 16.3493142857143],
 [-5.19883433098592, 16.8571053571429],
 [-5.19883433098592, 17.3648964285714],
 [-6.00642024647887, 17.7711292857143],
 [-5.90547200704225, 17.8726875],
 [-5.90547200704225, 18.7867114285714],
 [-5.75404964788732, 19.5483980357143],
 [-5.55215316901408, 19.5483980357143],
 [-4.5931448943662, 20.1069682142857],
 [-4.34077429577465, 20.8178757142857],
 [-3.78555897887324, 21.6303414285714],
 [-3.93698133802817, 22.0873533928571],
 [-3.0789213028169, 22.5951444642857],
 [-2.42275774647887, 23.2552728571429],
 [-2.0694389084507, 24.1185176785714],
 [-1.76659419014085, 25.1340998214286],
 [-1.66564595070423, 25.7434491071429],
 [-0.555215316901408, 25.8450073214286],
 [0.403792957746479, 25.0833207142857],
 [0.403792957746479, 26.0989028571429]]



goal_list =[[0.252370598591549, 0.506232857142857],
 [-0.706637676056338, 1.82648964285714],
 [-0.706637676056338, 2.03038517857143],
 [-0.858060035211268, 3.19752553571429],
 [-0.302844718309859, 4.46700321428571],
 [-0.454267077464789, 5.99037642857143],
 [-0.757111795774648, 6.39660928571429],
 [-2.32180950704225, 6.75206303571429],
 [-4.49219665492958, 7.71686607142857],
 [-5.24930845070423, 8.63089],
 [-7.06637676056338, 9.84958857142857],
 [-8.47965211267606, 10.9667289285714],
 [-8.12633327464789, 10.2558214285714],
 [-6.91495440140845, 12.1346483928571],
 [-6.00642024647887, 14.0134753571429],
 [-5.80452376760563, 12.7947767857143],
 [-5.19883433098592, 14.67360375],
 [-5.19883433098592, 16.1461978571429],
 [-5.19883433098592, 16.9586635714286],
 [-6.1578426056338, 17.7203501785714],
 [-6.25879084507042, 18.7867114285714],
 [-4.89598961267606, 19.2945025],
 [-4.79504137323944, 20.5132010714286],
 [-4.79504137323944, 21.5287832142857],
 [-2.52370598591549, 22.49358625],
 [-1.96849066901408, 25.4387744642857],
 [-1.56469771126761, 25.8957864285714],
 [-0.807585915492958, 26.4543566071429]]
"""


goal_list=[
[	0.858060035211268	,	-0.253895535714286	],
[	2.11991302816901	,	-0.050779107142857	],
[	4.34077429577465	,	0.101558214285714	],
[	5.75404964788732	,	-1.06636125	],
[	6.6625838028169	,	-1.77726875	],
[	6.56163556338028	,	-2.38661803571429	],
[	5.50167904929578	,	-3.19908375	],
[	5.95594612676056	,	-4.31622410714286	],
[	6.20831672535211	,	-5.94115553571429	],
[	6.20831672535211	,	-5.94115553571429	]]


#sin obstaculos menor a 10
goal_list=[
[	0.858060035211268	,	-0.253895535714286	],
[	1.81706830985916	,	-0.507791071428571	],
[	4.0884036971831	,	-0.660128392857143	],
[	6.35973908450704	,	-1.21869857142857	],
[	6.6625838028169	,	-2.03116428571429	],
[	6.35973908450704	,	-2.69129267857143	],
[	5.85499788732394	,	-2.99596732142857	],
[	6.1578426056338	,	-5.02713160714286	],
[	6.20831672535211	,	-5.94115553571429	],
[	6.20831672535211	,	-5.94115553571429	]]

#con obstaculos menor a 10
goal_list=[
[	0.509223801065719	,	0	],
[	1.37490426287744	,	-0.199903690685413	],
[	2.39335186500888	,	-0.399807381370826	],
[	2.85165328596803	,	-0.499759226713532	],
[	3.51364422735346	,	-0.699662917398946	],
[	4.32840230905862	,	-0.899566608084359	],
[	4.83762611012433	,	-1.09947029876977	],
[	5.49961705150977	,	-1.24939806678383	],
[	6.26345275310835	,	-1.29937398945518	],
[	6.61990941385435	,	-1.99903690685413	],
[	6.51806465364121	,	-2.44882021089631	],
[	6.36529751332149	,	-2.59874797891037	],
[	5.85607371225577	,	-3.8481460456942	],
[	5.4486946714032	,	-4.79768857644991	],
[	6.21253037300178	,	-5.89715887521968	],
[	6.21253037300178	,	-5.89715887521968	]]
#con obstaculos mayor a 10
"""
goal_list=[
#[	0	,	0	],
[	-0.865680461811723	,	-1.19942214411248	],
[	-0.814758081705151	,	-1.74915729349736	],
[	-0.814758081705151	,	-2.39884428822496	],
[	-0.509223801065719	,	-2.9985553602812	],
[	-0.203689520426288	,	-3.64833866432337	],
[	0.661990941385435	,	-3.69831458699473	],
[	1.42582664298401	,	-2.49879613356766	],
[	1.68043854351687	,	-2.2489165202109	],
[	2.34242948490231	,	-2.04901282952548	],
[	2.49519662522203	,	-2.14896467486819	],
[	3.92102326820604	,	-2.79865166959578	],
[	4.12471278863233	,	-2.89860351493849	],
[	5.34684991119005	,	-3.69821827768014	],
[	5.60146181172291	,	-3.8481460456942	],
[	6.31437513321492	,	-4.29792934973638	],
[	7.17452131438721	,	-4.49788119507909	],
[	7.68374511545293	,	-3.69807381370826	],
[	8.24942557726465	,	-3.64829050966608	],
[	8.91141651865009	,	-3.34838681898067	],
[	9.72617460035524	,	-2.94867574692443	],
[	10.1390879218472	,	-2.54882021089631	],
[	10.9992341030195	,	-1.34934991212654	],
[	11.3556907637655	,	-1.39932583479789	],
[	12.474138365897	,	-1.5992295254833	],
[	13.5944307282416	,	-2.09898875219684	],
[	13.8490426287744	,	-2.29889244288225	],
[	13.8490426287744	,	-2.29889244288225	]]



#mas de 20
goal_list=[
[	-0.763835701598579	,	0.999518453427065	],
[	-0.509223801065719	,	2.04901282952548	],
[	0.203689520426288	,	2.64872390158172	],
[	0.967525222024867	,	3.14848312829525	],
[	1.57859378330373	,	3.39836274165202	],
[	2.74980852575488	,	4.09802565905097	],
[	3.10626518650089	,	4.29792934973638	],
[	3.61548898756661	,	5.49735149384886	],
[	3.61548898756661	,	6.59682179261863	],
[	3.51364422735346	,	7.6962920913884	],
[	3.51364422735346	,	8.49590685413005	],
[	3.51364422735346	,	9.19547346221441	],
[	3.34179946714032	,	10.8947992970123	],
[	3.36087708703375	,	11.9942214411248	],
[	3.36087708703375	,	13.443523198594	],
[	3.36087708703375	,	13.7433787346221	],
[	3.46272184724689	,	15.1926804920914	],
[	3.46272184724689	,	15.4425601054482	],
[	4.58301420959147	,	16.2921507908612	],
[	5.39777229129663	,	16.2421748681898	],
[	6.11068561278863	,	16.0422230228471	],
[	7.23097797513322	,	16.0422711775044	],
[	8.24942557726465	,	15.992295254833	],
[	9.11510603907638	,	15.9423193321617	],
[	9.9807865008881	,	15.5924878734622	],
[	10.3881655417407	,	15.3925841827768	],
[	11.5084579040853	,	14.6929212653779	],
[	12.0686040852575	,	14.3430898066784	],
[	12.8833621669627	,	13.8933065026362	],
[	13.7490426287744	,	13.443523198594	],
[	14.3601111900533	,	13.1436676625659	],
[	15.0730245115453	,	12.7938362038664	],
[	15.7859378330373	,	12.444004745167	],
[	16.4479287744227	,	12.1441492091388	],
[	17.1099197158082	,	11.8942695957821	],
[	18.077444937833	,	11.744341827768	],
[	18.8922030195382	,	11.744341827768	],
[	19.5541939609236	,	12.4939806678383	],
[	19.8597282415631	,	13.6434268892794	],
[	19.8597282415631	,	13.6434268892794	]]
"""
"""
goal_list=[
[	-0.916602841918295	,	1.24939806678383	],
[	-0.25461190053286	,	1.94906098418278	],
[	-0.101844760213144	,	2.54877205623902	],
[	0.661990941385435	,	2.89860351493849	],
[	1.32398188277087	,	3.34838681898067	],
[	2.79704138543517	,	3.8481460456942	],
[	3.46272184724689	,	4.89764042179262	],
[	3.45272184724689	,	5.99711072056239	],
[	3.45179946714032	,	7.14655694200352	],
[	3.43179946714032	,	8.29595500878735	],
[	3.39364422735346	,	9.69528084358524	],
[	3.41272184724689	,	10.6947992970123	],
[	3.40272184724689	,	11.2945585237258	],
[	3.4079946714032	,	12.0940288224956	],
[	3.35272184724689	,	13.0935713532513	],
[	3.32272184724689	,	14.293113884007	],
[	3.35272184724689	,	15.14270456942	],
[	3.47272184724689	,	16.0923915641476	],
[	4.37932468916519	,	16.0923915641476	],
[	5.55053943161634	,	15.9923915641476	],
[	6.56898703374778	,	15.9923915641476	],
[	7.74020177619893	,	16.1923915641476	],
[	8.50403747779751	,	16.2923915641476	],
[	9.47156269982238	,	16.2923915641476	],
[	10.4863207815275	,	15.5525360281195	],
[	10.8464669626998	,	15.3426564147627	],
[	11.5593802841918	,	14.9428008787346	],
[	12.374138365897	,	14.5429934973638	],
[	12.9342845470693	,	14.3430898066784	],
[	13.7490426287744	,	14.0932101933216	],
[	14.2073440497336	,	13.8933065026362	],
[	15.0730245115453	,	13.443523198594	],
[	15.5313259325044	,	13.1936435852373	],
[	16.1423944937833	,	12.8937880492091	],
[	16.9062301953819	,	12.5439565905097	],
[	17.7719106571936	,	11.9942214411248	],
[	18.5866687388988	,	11.6443899824253	],
[	18.5940477797513	,	11.6943177504394	],
[	19.3569611012433	,	12.7438602811951	],
[	19.5597282415631	,	13.6434268892794	],
[	19.5597282415631	,	13.6434268892794	]]
"""
"""
goal_list=[
#[	0	,	0	],
[	1.12029236234458	,	0.449735149384886	],
[	1.68043854351687	,	0.949542530755712	],
[	2.74980852575488	,	1.44930175746924	],
[	4.12471278863233	,	1.89908506151142	],
[	4.93947087033748	,	1.99903690685413	],
[	5.85607371225577	,	2.04901282952548	],
[	7.02728845470693	,	3.54829050966608	],
[	8.14758081705151	,	5.14752003514939	],
[	9.11695079928952	,	6.59686994727592	],
[	9.6317088809947	,	7.7962920913884	],
[	11.304768383659	,	7.74626801405975	],
[	12.5269055062167	,	6.89667732864675	],
[	13.443508348135	,	6.1970144112478	],
[	14.1582664298401	,	5.59730333919157	],
[	14.982248312611	,	5.43739964850615	],
[	17.008074955595	,	5.43747188049209	],
[	18.6375911190053	,	5.24752003514939	],
[	19.2995820603908	,	5.04756818980668	],
[	20.1634177619893	,	4.52783304042179	],
[	21.1327877442274	,	3.9081460456942	],
[	22.5986143872114	,	2.9985553602812	],
[	23.7807515097691	,	2.19894059753954	],
[	25.0538110124334	,	1.5992295254833	],
[	25.5630348134991	,	1.19942214411248	],
[	26.3268705150977	,	0.649686994727592	],
[	26.7342495559503	,	0	],
[	26.7342495559503	,	0	]]
"""

goal_list=[
[	-0.916602841918295	,	1.24939806678383	],
[	-0.25461190053286	,	1.94906098418278	],
[	-0.101844760213144	,	2.54877205623902	],
[	0.661990941385435	,	2.89860351493849	],
[	1.32398188277087	,	3.34838681898067	],
[	2.59704138543517	,	3.8481460456942	],
[	3.46272184724689	,	4.89764042179262	],
[	3.46272184724689	,	5.99711072056239	],
[	3.46179946714032	,	7.14655694200352	],
[	3.41179946714032	,	8.29595500878735	],
[	3.35364422735346	,	9.39528084358524	],
[	3.30272184724689	,	10.8947992970123	],
[	3.36272184724689	,	11.2945585237258	],
[	3.28179946714032	,	12.3940288224956	],
[	3.35272184724689	,	13.2935713532513	],
[	3.35272184724689	,	14.193113884007	],
[	3.35272184724689	,	15.24270456942	],
[	3.40272184724689	,	15.89915641476	],
[	4.37932468916519	,	16.0923915641476	],
[	5.55053943161634	,	16.0423193321617	],
[	6.86898703374778	,	16.1523193321617	],
[	7.74020177619893	,	16.152295254833	],
[	8.50403747779751	,	16.152367486819	],
[	9.47156269982238	,	15.992367486819	],
[	10.2863207815275	,	15.7925360281195	],
[	10.8464669626998	,	15.7426564147627	],
[	11.6093802841918	,	15.0928008787346	],
[	12.3741383658970	,	14.5429934973638	],
[	12.9342845470693	,	14.3430898066784	],
[	13.7490426287744	,	14.0932101933216	],
[	14.2073440497336	,	13.8933065026362	],
[	15.0730245115453	,	13.443523198594	],
[	15.5313259325044	,	13.1936435852373	],
[	16.1423944937833	,	12.8937880492091	],
[	16.9062301953819	,	12.5439565905097	],
[	17.7719106571936	,	11.9942214411248	],
[	18.5866687388988	,	11.6443899824253	],
[	18.5940477797513	,	11.6943177504394	],
[	19.2069611012433	,	12.8438602811951	],
[	19.5597282415631	,	13.6434268892794	],
[	19.5597282415631	,	13.6434268892794	]]









datos=[]
class GazeboEnv:
    """Superclass for all Gazebo environments."""

    def __init__(self, launchfile, environment_dim):
        self.environment_dim = environment_dim
        self.odom_x = 0
        self.odom_y = 0
        self.i=0
        self.goal_x = 0
        self.goal_y = 0
        self.collision=True

        self.upper = 5.0+10
        self.lower = -5.0+10
        self.velodyne_data = np.ones(self.environment_dim) * 10
        self.last_odom = None

        self.set_self_state = ModelState()
        self.set_self_state.model_name = "turtlebot3_burger"
        self.set_self_state.pose.position.x = 0.0
        self.set_self_state.pose.position.y = 0.0
        self.set_self_state.pose.position.z = 0.0
        self.set_self_state.pose.orientation.x = 0.0
        self.set_self_state.pose.orientation.y = 0.0
        self.set_self_state.pose.orientation.z = 0.0
        self.set_self_state.pose.orientation.w = 1.0
        """
        self.gaps = [[0 - 0.03, 0 + np.pi / self.environment_dim]]
        for m in range(0,int(self.environment_dim/2-2)):
            self.gaps.append(
                [self.gaps[m][1], self.gaps[m][1] + np.pi / self.environment_dim]
            )

        self.gaps.append([3*np.pi/2,3*np.pi/2+np.pi / self.environment_dim])
        for m in range(len(self.gaps)-1 ,self.environment_dim ):
            self.gaps.append(
                [self.gaps[m][1], self.gaps[m][1] + np.pi / self.environment_dim]
            )
        """
        self.gaps = [[3*np.pi/2, 3*np.pi/2 + np.pi / self.environment_dim]]
        for m in range(0,int(self.environment_dim/2-1)):
            print(m)
            self.gaps.append(
                [self.gaps[m][1], self.gaps[m][1] + np.pi / self.environment_dim]
            )
        
        self.gaps.append([0-0.03, 0 + np.pi / self.environment_dim])
        for m in range(len(self.gaps)-1 ,self.environment_dim-1 ):
            print(m)
            self.gaps.append(
                [self.gaps[m][1], self.gaps[m][1] + np.pi / self.environment_dim]
            )
        print(len(self.gaps))
        #print('*************************************')
        self.gaps[-1][-1] += 0.03
        port = "11311"
        subprocess.Popen(["roscore", "-p", port])

        print("Roscore launched!")

        # Launch the simulation with the given launchfile name
        rospy.init_node("gym", anonymous=True)
        if launchfile.startswith("/"):
            fullpath = launchfile
        else:
            fullpath = os.path.join(os.path.dirname(__file__), "assets", launchfile)
        if not path.exists(fullpath):
            raise IOError("File " + fullpath + " does not exist")

        subprocess.Popen(["roslaunch", "-p", port, fullpath])
        print("Gazebo launched!")

        # Set up the ROS publishers and subscribers
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.set_state = rospy.Publisher(
            "gazebo/set_model_state", ModelState, queue_size=10
        )
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.reset_proxy = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        self.publisher = rospy.Publisher("goal_point", MarkerArray, queue_size=3)
        self.publisher2 = rospy.Publisher("linear_velocity", MarkerArray, queue_size=1)
        self.publisher3 = rospy.Publisher("angular_velocity", MarkerArray, queue_size=1)
        self.odom = rospy.Subscriber(
            "odom", Odometry, self.odom_callback, queue_size=1
        )
        self.velodyne = rospy.Subscriber(
            "scan", LaserScan, self.velodyne_callback, queue_size=1
        )


    # Read velodyne pointcloud and turn it into distance data, then select the minimum value for each angle
    # range as state representation
    def velodyne_callback(self, lidar_data):
        self.velodyne_data = np.ones(self.environment_dim) * 10
        angles = np.arange(lidar_data.angle_min, lidar_data.angle_max, lidar_data.angle_increment)
        ranges = np.array(lidar_data.ranges)
        self.lidar_data = np.ones(self.environment_dim) * 10


        angles = np.arange(lidar_data.angle_min, lidar_data.angle_max + lidar_data.angle_increment, lidar_data.angle_increment)
        ranges = np.array(lidar_data.ranges)
        #print(self.gaps)
        #print()
        for i in range(len(angles)):
            if lidar_data.range_min < ranges[i] < lidar_data.range_max:
                # Calcular el ángulo azimutal en coordenadas polares
                angle = angles[i]
                # Actualizar los datos del LiDAR en función de los ángulos de medición
                for j in range(len(self.gaps)):
                    if self.gaps[j][0] <= angles[i] < self.gaps[j][1]:
                        self.lidar_data[j] = min(self.lidar_data[j], ranges[i])
                        break
        self.velodyne_data= self.lidar_data 
        #print(self.velodyne_data)                  
        

    def odom_callback(self, od_data):

        self.last_odom = od_data

    # Perform an action and read a new state
    def preprocess_lidar_distances(self, scan_range):
        return scan_range
    def step(self, action):
        target = False

        # Publish the robot action
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.angular.z = action[1]
        self.vel_pub.publish(vel_cmd)
        self.publish_markers(action)

        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        # propagate state for TIME_DELTA seconds
        time.sleep(TIME_DELTA)

        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            pass
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")
                        
        scan_range = []
        

        done, self.collision, min_laser = self.observe_collision(self.velodyne_data)
        v_state = []
        v_state[:] = self.velodyne_data[:]
        laser_state = [v_state]

        # Calculate robot heading from odometry data
        self.odom_x = self.last_odom.pose.pose.position.x
        self.odom_y = self.last_odom.pose.pose.position.y
        quaternion = Quaternion(
            self.last_odom.pose.pose.orientation.w,
            self.last_odom.pose.pose.orientation.x,
            self.last_odom.pose.pose.orientation.y,
            self.last_odom.pose.pose.orientation.z,
        )
        euler = quaternion.to_euler(degrees=False)
        angle = round(euler[2], 4)

        # Calculate distance to the goal from the robot
        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )

        # Calculate the relative angle between the robots heading and heading toward the goal
        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y
        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))
        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle
        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        # Detect if the goal has been reached and give a large positive reward
        if self.i >= (len(goal_list)):
            
            self.collision = True
            target = True
            self.reset()
            self.i=0
            done = True

        if distance < GOAL_REACHED_DIST:
            target = True
            
            self.change_goal()
            self.i=self.i+1
            done = False

        robot_state = [distance, theta, action[0], action[1]]
        state = np.append(laser_state, robot_state)
        reward = self.get_reward(target, self.collision, action, min_laser,distance)
        return state, reward, done, target

    def reset(self):
        
        

        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service("/gazebo/reset_world")
        try:
            self.reset_proxy()

        except rospy.ServiceException as e:
            print("/gazebo/reset_simulation service call failed")
        time.sleep(7)
        sleep_time = 7  # en segundos
        rospy.loginfo("Esperando %d segundos...", sleep_time)
        angle = np.random.uniform(-np.pi, np.pi)
        quaternion = Quaternion.from_euler(0.0, 0.0, 0*1*np.pi/2)
        object_state = self.set_self_state
        if self.collision is True:
            x=0
            y=0
            datos.append(self.i)

            # Nombre del archivo CSV
            nombre_archivo = 'sin_obs_m20_11_2.csv'

            # Abrir el archivo CSV en modo escritura para borrar contenido previo
            with open(nombre_archivo, mode='w', newline='') as archivo:
                escritor = csv.writer(archivo)

                # Escribir los datos en una columna
                for dato in datos:
                    escritor.writerow([dato])  # Cada dato en una nueva fila

            # Cerrar el archivo
            archivo.close()
            self.i=0
        else:
            x = self.goal_x
            y = self.goal_y
            x = self.last_odom.pose.pose.position.x        
            y = self.last_odom.pose.pose.position.y
        position_ok = False
        while not position_ok:
            #x = np.random.uniform(-4.5, 4.5)
            #y = np.random.uniform(-4.5, 4.5)
            position_ok = check_pos(x, y)
        object_state.pose.position.x = x
        object_state.pose.position.y = y
        object_state.pose.position.z = 0.12

        object_state.pose.orientation.x = quaternion.x
        object_state.pose.orientation.y = quaternion.y
        object_state.pose.orientation.z = quaternion.z
        object_state.pose.orientation.w = quaternion.w
        self.set_state.publish(object_state)

        self.odom_x = object_state.pose.position.x
        self.odom_y = object_state.pose.position.y
        
        # set a random goal in empty space in environment
        
        
        # randomly scatter boxes in the environment
        #self.random_box()
        #self.publish_markers([0.0, 0.0])

        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        time.sleep(TIME_DELTA)

        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")
        v_state = []
        v_state[:] = self.velodyne_data[:]
        laser_state = [v_state]

        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )

        
        

        self.change_goal()
        self.i=0
        

        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y

        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))

        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle

        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta
        
        robot_state = [distance, theta, 0.0, 0.0]
        state = np.append(laser_state, robot_state)
        return state

    def change_goal(self):
        # Place a new goal and check if its location is not on one of the obstacles
        print(self.i)

        if self.upper < 10 +10:
            self.upper += 0.004
        if self.lower > -10+10:
            self.lower -= 0.004

        goal_ok = False

        while not goal_ok:

            #self.goal_x = self.odom_x + random.uniform(self.upper, self.lower)
            #self.goal_y = self.odom_y + random.uniform(self.upper, self.lower)
            self.goal_x =goal_list[self.i][0]-0.1
            self.goal_y =goal_list[self.i][1]+0.1
            # self.odom_x + #random.uniform(self.upper, self.lower)
            # self.odom_y + #random.uniform(self.upper, self.lower)
            print(self.goal_x,self.goal_y)

            goal_ok = check_pos(self.goal_x, self.goal_y)

    def random_box(self):
        # Randomly change the location of the boxes in the environment on each reset to randomize the training
        # environment
        for i in range(4):
            name = "cardboard_box_" + str(i)

            x = 0
            y = 0
            box_ok = False
            while not box_ok:
                x = np.random.uniform(-6, 6)
                y = np.random.uniform(-6, 6)
                box_ok = check_pos(x, y)
                distance_to_robot = np.linalg.norm([x - self.odom_x, y - self.odom_y])
                distance_to_goal = np.linalg.norm([x - self.goal_x, y - self.goal_y])
                if distance_to_robot < 1.5 or distance_to_goal < 1.5:
                    box_ok = False
            box_state = ModelState()
            box_state.model_name = name
            box_state.pose.position.x = x
            box_state.pose.position.y = y
            box_state.pose.position.z = 0.0
            box_state.pose.orientation.x = 0.0
            box_state.pose.orientation.y = 0.0
            box_state.pose.orientation.z = 0.0
            box_state.pose.orientation.w = 1.0
            self.set_state.publish(box_state)

    def publish_markers(self, action):
        # Publish visual data in Rviz
        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
        marker.pose.position.z = 0

        markerArray.markers.append(marker)

        self.publisher.publish(markerArray)

        markerArray2 = MarkerArray()
        marker2 = Marker()
        marker2.header.frame_id = "odom"
        marker2.type = marker.CUBE
        marker2.action = marker.ADD
        marker2.scale.x = abs(action[0])
        marker2.scale.y = 0.1
        marker2.scale.z = 0.01
        marker2.color.a = 1.0
        marker2.color.r = 1.0
        marker2.color.g = 0.0
        marker2.color.b = 0.0
        marker2.pose.orientation.w = 1.0
        marker2.pose.position.x = 5
        marker2.pose.position.y = 0
        marker2.pose.position.z = 0

        markerArray2.markers.append(marker2)
        self.publisher2.publish(markerArray2)

        markerArray3 = MarkerArray()
        marker3 = Marker()
        marker3.header.frame_id = "odom"
        marker3.type = marker.CUBE
        marker3.action = marker.ADD
        marker3.scale.x = abs(action[1])
        marker3.scale.y = 0.1
        marker3.scale.z = 0.01
        marker3.color.a = 1.0
        marker3.color.r = 1.0
        marker3.color.g = 0.0
        marker3.color.b = 0.0
        marker3.pose.orientation.w = 1.0
        marker3.pose.position.x = 5
        marker3.pose.position.y = 0.2
        marker3.pose.position.z = 0

        markerArray3.markers.append(marker3)
        self.publisher3.publish(markerArray3)

    @staticmethod
    def observe_collision(laser_data):
        # Detect a collision from laser data
        min_laser = min(laser_data)
        if min_laser < COLLISION_DIST:
            return True, True, min_laser
        return False, False, min_laser

    @staticmethod
    def get_reward(target, collision, action, min_laser,distance):
        if target:
            return 100.0
        elif collision:
            return -100.0-distance
        else:
            r3 = lambda x: 1 - x if x < 1 else 0.0
            return action[0] / 2 - abs(action[1]) / 2 - r3(min_laser) / 2
