import numpy as np
import scipy as sp
import scipy.misc as spm
import math as ma
import sys
import time
import matplotlib.pyplot as plt

def Dijkst(ist,isp,wei):
    # Dijkstra algorithm for shortest path in a graph
    #    ist: index of starting node
    #    isp: index of stopping node
    #    wei: weight matrix

    # exception handling (start = stop)
    if (ist == isp):  #Trivial case that we only have 1 node, e.g. start = stop
        shpath = [ist]
        return shpath 
    # initialization
    N         =  len(wei)
    Inf       =  sys.maxint
    UnVisited =  np.ones(N,int)
    cost      =  np.ones(N)*1.e6 #Weight
    par       = -np.ones(N,int)*Inf #Parent node

    ##START##
    # set the source point and get its (unvisited) neighbors
    jj            = ist #start node 
    cost[jj]      = 0
    UnVisited[jj] = 0
    tmp           = UnVisited*wei[jj,:] #wei[jj,:] gives the weights of the edges attatched to jj, if we have visited =0
    ineigh        = np.array(tmp.nonzero()).flatten()
    L             = np.array(UnVisited.nonzero()).flatten().size
    # start Dijkstra algorithm
    while (L != 0):
        # step 1: update cost of unvisited neighbors,
        #         compare and (maybe) update
        for k in ineigh:
            newcost = cost[jj] + wei[jj,k]
            if ( newcost < cost[k] ):
                cost[k] = newcost
                par[k]  = jj

        # step 2: determine minimum-cost point among UnVisited
        #         vertices and make this point the new point
        icnsdr     = np.array(UnVisited.nonzero()).flatten()
        cmin,icmin = cost[icnsdr].min(0),cost[icnsdr].argmin(0) #Lowest cost and index of lowst cost
        jj         = icnsdr[icmin] #Move on to next point

        # step 3: update "visited"-status and determine neighbors of new point
        UnVisited[jj] = 0
        tmp           = UnVisited*wei[jj,:]
        ineigh        = np.array(tmp.nonzero()).flatten()
        L             = np.array(UnVisited.nonzero()).flatten().size

    # determine the shortest path
    shpath = [isp]
    while par[isp] != ist:
        shpath.append(par[isp])
        isp = par[isp]
    shpath.append(ist)

    return shpath[::-1] #reversing a list

def calcWei(RX,RY,RA,RB,RV):
    # calculate the weight matrix between the points

    n    = len(RX)
    wei = np.zeros((n,n),dtype=float)
    m    = len(RA)
    for i in range(m):
        xa = RX[RA[i]-1]
        ya = RY[RA[i]-1]
        xb = RX[RB[i]-1]
        yb = RY[RB[i]-1]
        dd = ma.sqrt((xb-xa)**2 + (yb-ya)**2)
        tt = dd/RV[i] #time taken
        wei[RA[i]-1,RB[i]-1] = tt
    return wei
#####################Changing Weights###################################################
def UpdateWei(wei0, c, xi, truefalse):
    List = [c  for i in c] 
    #Making a 'matrix' M with each row = c
    Mat= np.array(List)
    MatT= np.transpose(Mat)
    wei= (xi*.5*(Mat+MatT)+wei0)*truefalse
    #0.5(M+M^T)[i][j]=(c[i]+c[j])/2 as required
    #add original weight(wei0), and make zero if wei0=0(truefalse)
    return wei
######################Adding cars####################################################
def caradd(c): #Adds new cars to the system at node 13
    c[12] += 20
    return c
#######################Shifting cars###################################
def cars(L, c, cmx, wei, used):
#Uses c=number of cars at each node
    cnew=np.zeros(L, dtype=int)
    #Creates 'dummy' vector so we don't change the c[i] when running through for loop
    #If we don't have this cars will move more than one node per time step.
    for i in range(L):
        shpath = Dijkst(i,51, wei)
   		#run Dijkstra's     
        if len(shpath)==1 :
            cnew[i]+= c[i]*6/10
            #Cars leaving system
        else:
        	cnew[shpath[1]] += c[i] - c[i]*3/10
        	cnew[i] += c[i]*3/10
        	#Moving 70% of cars to next node
        	#Not using c[i]*7/10, as integar division truncates instead of rounding.
        	#If this was not done, the number of cars would not be conserved
        ############Changing used matrix##########
        	if c[i]!=0:
				used[i][shpath[1]] = False
				used[shpath[1]][i] = False
				#If c[i]=0, that implies we will move cars to shpath[1], and therefore the road connecting the two will be used.
        ##########################################
    cmax= np.maximum(cmx, cnew)
    #recalulate max
    return cnew, cmax, used
    #puts out new, cmax, and used
##################Main Function####################################################
def flow(t=199, xi=0.01, TakeOut=-1):
		#state node you would like to remove from the system(not 53/13)
	if TakeOut >= 0:    
		for i in range(n):
			wei0[TakeOut-1][i]=900
			wei0[i][TakeOut-1]=900
            #Removing all edges to and from this node
	c=np.zeros(n, dtype=int)
	cmax=np.zeros(n, dtype=int)
	truefalse = wei0 > 0
	used = wei0 > 0
	wei = UpdateWei(wei0, c, xi,truefalse)
	#Setting up variables
	for i in range(t):
		if i<=179:
		#cycle through steps where we add cars
			c=caradd(c)
			c, cmax, used = cars(n, c, cmax, wei, used)
		else:
		#cycle through 'cool down' steps
			c, cmax, used = cars(n, c, cmax, wei, used)    
		wei = UpdateWei(wei0, c, xi,truefalse)
		#Update weights	
	return cmax ,used
##########################Maximum Node Analysis###############################
def plot(cmax):
		sort_index = np.argsort(cmax)[-5:]
		busy = [[i+1,cmax[i]]  for i in sort_index]
		busy = busy[::-1]
		print busy
		#Compiling and printing the 5 busiest nodes.
		a= .5*(cmax)**2 + 1
		plt.scatter(RomeX, RomeY, s=a, color='blue', edgecolors='black' , alpha=.75 ,zorder=2)
		#Creating scatterplot, with varying circle radius
		Xbusy =[RomeX[i[0]-1] for i in busy]
		Ybusy =[RomeY[i[0]-1] for i in busy]
		abusy =[a[i[0]-1] for i in busy]
		plt.scatter(Xbusy, Ybusy, s=abusy, color='red', edgecolors='black' , alpha=.75,zorder=3)
		#Creating red overlay for scatter plot, 5 greatest maxima only.
		pic =spm.imread('Rome.png')
		plt.imshow(pic, zorder=1,extent=(0.04,15.8,.1,11.17)) 
		#Importing and setting background.
		plt.show()
##########################Non-Taversed Roads##################################
def roads(used):
		for k in range(n):
			for j in range(k):
				#Only loop over top traingle of matrix, so we do not repeat edges
				if used[j][k]!=0 and used[k][j]!=0:
					#This is true for a two-way road that hasn't be used
					print '(' +str(j+1)+'<=>'+ str(k+1)+')'
				elif used[j][k]!=0:
					#True for one way j->k
					print '(' +str(j+1)+'-->'+ str(k+1)+')'
				elif used[k][j]!=0:
					#True for one way k->j
					print '(' +str(k+1)+'-->'+ str(j+1)+')'

#########################################################################
#Initialization of dijkstra and contruction of weight matrix
#(Unchanged from problem classes)
if __name__ == '__main__':

    import numpy as np
    import scipy as sp
    import csv

    RomeX = np.empty(0,dtype=float)
    RomeY = np.empty(0,dtype=float)
    with open('RomeVertices','r') as file:
        AAA = csv.reader(file)
        for row in AAA:
            RomeX = np.concatenate((RomeX,[float(row[1])]))
            RomeY = np.concatenate((RomeY,[float(row[2])]))
    file.close()

    RomeA = np.empty(0,dtype=int)
    RomeB = np.empty(0,dtype=int)
    RomeV = np.empty(0,dtype=float)
    with open('RomeEdge','r') as file:
        AAA = csv.reader(file)
        for row in AAA:
            RomeA = np.concatenate((RomeA,[int(row[0])]))
            RomeB = np.concatenate((RomeB,[int(row[1])]))
            RomeV = np.concatenate((RomeV,[float(row[2])]))
    file.close()

    wei0 = calcWei(RomeX,RomeY,RomeA,RomeB,RomeV)

    ist = 12 # St. Peter's Square
    isp = 51 # Coliseum
    n=len(RomeX)


    input = sys.argv[1]
    #Allows me to choose which part of the coursework to answer
    if input == '1':
    	cmax, used=flow()
    	plot(cmax)
    	#Does maximum node analysis for unchanged graph
    if input == '2':
    	cmax, used=flow()
    	roads(used)  	
    	#Finds untraversed redges for unchanged graph
    if input =='3':
    	cmax, used=flow(xi=0)
    	plot(cmax)
    	#Does maximum node analysis for graph without node 30 
    if input == '4':
    	cmax0, used0=flow()
    	cmax1, used1=flow(TakeOut=30)
    	#Find maximums for the normal system and system without node 30
    	diff= cmax1- cmax0
    	#calculate the difference
    	diff_index = np.argsort(diff)
    	#sort the indices in terms of the maximum
    	increase, decrease = [], []
    	for i in diff_index:
    		if diff[i]>0:
    			increase.append([i+1,diff[i]])
    			#Create list of increased maxima
    		elif diff[i]<0:
    			decrease.append([i+1, -diff[i]])
    			#List of decreased maxima
    	increase=increase[::-1]
    	print np.array(increase)
    	print np.array(decrease[1:])
    	#print lists removing node 30
    	plot(cmax1)
    	#plot the system without node 30