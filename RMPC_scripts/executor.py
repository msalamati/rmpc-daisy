import sys
import shlex
import shutil
import subprocess
import time
from decimal import *
import os
from copy import deepcopy
from utils import *
from error_computation_tool import *

def checkInput(F,G,HXK,NumControllers,dimensions):
	if len(F)!=NumControllers:
		exit("F != NumControllers")
	if len(G)!=len(F):
		exit("G != NumControllers != F")
	if len(G)!=len(HXK):
		exit("G != HXK != F")
	if len(F[0])!=dimensions:
		exit("F[0] != dimensions")
	if isinstance(G, basestring):
		exit("G[0] is not a string")
	return

def getMatrix(linesKickOff):
	ret=[]
	guard=False
	for line in linesKickOff:
		if not line.isspace():
			ret.append(line)
			guard=True
		elif not guard:
			continue
		else:
			break
	return ret

def getMatrixFromPattern(pattern,lines):
	mat=[]
	for i in range(0,len(lines)):
		if pattern in lines[i]:
			mat=getMatrix(lines[i+1:])
			return mat

def encodeEdgeFor_H_(i,dimensions,values):
	res=""
	X="X"
	for index,value in enumerate(values[i:i+dimensions]):
		res=res+"("+value+"*"+X+str(index)+")+"
	res=res[:-1]
	return res
	
def encodeEdgeFor_K_(prev_from_H,K_val,symbol="<="):
	res=prev_from_H+" "+symbol+" ("+K_val+")"
	return res

def checkforAllZeros(res,dimensions):
	if res.count("0*")==dimensions and (("(0)" in res) or ("(0.000000000000000)" in res) or ("(0.0)" in res)):
		return True
	return False

def processBounds(listBounds,epsilon,dimensions):
	x_i=0
	i=0
	X="X"
	finalList=[]
	while i<len(listBounds):
		finalList.append(X+str(x_i)+">= "+listBounds[i]+"-"+epsilon)
		finalList.append(X+str(x_i)+"<= "+listBounds[i+1]+"+"+epsilon)
		i=i+2
		x_i=x_i+1
	return finalList

def scanOutputMatlabFor_HX_lt_K(namefile,dimensions):
	f= open(namefile)
	lines=f.readlines()
	f.close()
	HK={}
	HK_raw=getMatrixFromPattern("LINE_RANGE = ",lines)
	reg_Lim=getMatrixFromPattern("region_pointer =",lines)
	region=0
	start=0
	for reg_line_raw in reg_Lim:
		reg_line=reg_line_raw.strip()
		end=int(float(reg_line))
		for index,line in enumerate(HK_raw[start:end+1]):
			values=line.split()
			res=encodeEdgeFor_H_(0,dimensions,values)
			res=encodeEdgeFor_K_(res,values[dimensions],symbol="-")
			bounds=processBounds(values[dimensions+1:],epsilon,dimensions)
			if region in HK:
				HK[region].append((res,bounds))
			else:
				HK[region]=[(res,bounds)]
		region=region+1
		start=end+1
	return HK
	
def scanOutputMatlabFor_F(namefile):
	f= open(namefile)
	lines=f.readlines()
	f.close()
	F_raw=getMatrixFromPattern("F =",lines)
	F={}
	for index,line in enumerate(F_raw):
		values=line.split()
		for value in values:
			if index in F:
				F[index].append(value.strip())
			else:
				F[index]=[value.strip()]
	return F

#Vertices -0.031675883117556  0.406593450969933  -3.179766281717767  0.390368411060607  
def processVertices(vertices_raw,epsilon,dimensions):
	if isinstance(vertices_raw, basestring): 
		V_line_raw=vertices_raw.strip()
		V_line_raw=V_line_raw.split()
	if isinstance(vertices_raw, list): 
		V_line_raw=vertices_raw
	x_i=0
	i=0
	X="X"
	finalList=[]
	while x_i<dimensions:
		minVal=Decimal("+Infinity")
		maxVal=Decimal("-Infinity")
		i=x_i
		
		while i<len(V_line_raw):
			if Decimal(V_line_raw[i])>maxVal:
				maxVal=Decimal(V_line_raw[i])
			if Decimal(V_line_raw[i])<minVal:
				minVal=Decimal(V_line_raw[i])
			i=i+dimensions
			
		maxVal=str(maxVal)+" + "+epsilon
		minVal=str(minVal)+" - "+epsilon
		
		finalList.append(X+str(x_i)+">= "+minVal)
		finalList.append(X+str(x_i)+"<= "+maxVal)
		
		x_i=x_i+1
	return finalList
		

#[1.000000000000000 2.000000000000000 -0.005153862299491 0.999986718763503 0.406751304045955
#deltaX[(11,12)]=[["X0 >= -3.179766281717765","X0 <= 0.019456205461581"," X1 >= 0.4007","X1 <= 1.45"]]
def scanOutputMatlabForNeighbours(namefile,dimensions,epsilon):
	f= open(namefile)
	lines=f.readlines()
	f.close()
	N_raw=getMatrixFromPattern("neigh_mat =",lines)
	V_raw=getMatrixFromPattern("Vertices =",lines)
	deltaX={}
	for index,line in enumerate(N_raw):
		lineClean=line.strip()
		values=lineClean.split()
		res=encodeEdgeFor_H_(2,dimensions,values)
		res=encodeEdgeFor_K_(res,values[2+dimensions],"=")
		lb_res=res.replace("=",">=")
		ub_res=res.replace("=","<=")
		lb_res=lb_res+"-"+epsilon
		ub_res=ub_res+"+"+epsilon
		r1=int(round(float(values[0])))
		r2=int(round(float(values[1])))
		deltaX[(r1,r2)]=[lb_res,ub_res]
		deltaX[(r1,r2)]=deltaX[(r1,r2)]+processVertices(V_raw[index],epsilon,dimensions)
	return deltaX

def scanOutputMatlabFor_X_(namefile,epsilon):
	f= open(namefile)
	lines=f.readlines()
	f.close()
	X_raw=getMatrixFromPattern("X =",lines)
	X={}
	for index,line in enumerate(X_raw):
		values=line.split()
		strFinal=""
		X[index]=[]
		i=0
		j=0
		while j < len(values):
			X[index].append("X"+str(i)+">= "+values[j]+" - ("+epsilon+")")
			X[index].append("X"+str(i)+"<= "+values[j+1]+" + ("+epsilon+")")
			i=i+1
			j=j+2	
	return X

def scanOutputMatlabFor_G(namefile):
	f= open(namefile)
	lines=f.readlines()
	f.close()
	G_raw=getMatrixFromPattern("G =",lines)
	G={}
	for index,line in enumerate(G_raw):
		G[index]=line.strip()
	return G
	
def scanOutputMatlabForMaxUiUj(outputFile):
	f= open(outputFile)
	lines=f.readlines()
	f.close()
	maxUiUj_raw=getMatrixFromPattern("error_ij =",lines)
	maxUiUj=maxUiUj_raw[0].strip()
	try:
		float(maxUiUj)
	except:
		print "error_ij in wrong format"
		exit(0)
	return maxUiUj
	
########################################################################
#### INPUT PARAMETERS ##################################################
########################################################################

if len(sys.argv)<4:
	print "Arg-1 MATLAB delta\n"
	print "Arg-2 epsilon\n"
	print "Arg-3 filename for temporary files\n"
	exit("Please provide input parameters. Ex. python executor.py 0.1 0.001 pendulum")
	
matlabDelta=str(sys.argv[1]) #"0.1" max disturbance input to Matlab
epsilon=str(sys.argv[2]) #"0.001" size of the tubes
filename=str(sys.argv[3]) #"pendulum"

outputFile="outputMatlab.txt"
spaceForFPError="N/A"

########################################################################
########################################################################

print "###START###\n\n"

start = time.time()

while True:
	
	print "Robustness coefficient (delta) given to MATLAB: "+matlabDelta+"\n\n"	
	
	if os.path.exists("./output/"):
		shutil.rmtree("./output/", ignore_errors=True)
	os.mkdir('./output/')
	if os.path.exists("./inputFiles/"):
		shutil.rmtree("./inputFiles/", ignore_errors=True)
	os.mkdir('./inputFiles/')

	try:
		# give in input initial delta to matlab
		#exe="matlab -nodisplay -nosplash -r \"RMPC1("+initialDelta+");quit\" > outputMatlab.txt"
		exe="matlab -nodisplay -nosplash -nodesktop -r \"RMPC3("+str(matlabDelta)+", "+epsilon+"); quit\" > "+outputFile
		exe=shlex.split(exe)
		p=subprocess.Popen(exe,shell=False)
		p.wait()
	except:
		print "Error while calling matlab"
		exit(0)
	
	X=scanOutputMatlabFor_X_(outputFile,epsilon)
	F=scanOutputMatlabFor_F(outputFile)
	G=scanOutputMatlabFor_G(outputFile)
	
	dimensions=int(len(F[0]))
	numControllers=int(len(F))
	
	HXK=scanOutputMatlabFor_HX_lt_K(outputFile,dimensions)

	#deltaX=scanOutputMatlabForNeighbours(outputFile,dimensions,epsilon)
	
	maxUiUj= scanOutputMatlabForMaxUiUj(outputFile)
	
	checkInput(F,G,HXK,numControllers,dimensions)
	
	if (Decimal(matlabDelta)>=Decimal(maxUiUj)):
		spaceForFPError=str(Decimal(matlabDelta)-Decimal(maxUiUj))
		
		#checkControllersChoiseError(filename,dimensions,numControllers,tmp_deltaX,F,G,X,HXK,divLimits[i],totDivs[i])
		
		PrecisionTuningForFXG(filename,dimensions,numControllers,X,F,G,precision="0",delta=spaceForFPError)
		PrecisionTuningForHXK(filename,numControllers,dimensions,HXK,precision="0",delta=epsilon)

		failureC,maxErrorFailureC,maxValueCtr,maxPrecCtr=readMaxUniformPrecisionController(filename,numControllers)
		totPrecCtr=readMixedPrecisionConfigController(filename,numControllers)

		failureB,maxErrorFailureB,maxValueBrd,maxPrecBrd=readMaxUniformPrecisionBorders(filename,numControllers,HXK)
		totPrecBrd=readMixedPrecisionConfigBorders(filename,numControllers,HXK)
		
		if not failureC and not failureB:
			break
		else:
			print "UNABLE to satisfy the matlab bound (even with with Float64 precision). Asking MATLAB for new controller."
			matlabDelta=str(Decimal(Decimal(matlabDelta)+Decimal(maxErrorFailure)))
	else:
		print "UNABLE to satisfy the matlab bound because of max|Ui-Uj| = "+str(maxUiUj)+" > "+matlabDelta+". Asking MATLAB for new controller."
		matlabDelta=maxUiUj

end = time.time()


print "\n\nRESULTS: \n"
print "Execution time of the analysis: "+str((end - start)/60.0)+" min"
print "Delta = "+str(matlabDelta)
print "Max |Ui - Uj| = "+str(maxUiUj)
print "Space for FP error = " + str(spaceForFPError)
print "\n\n"
print "Precision for Controllers(F and G) UNIFORM: Float"+str(maxValueCtr)+", Total number of bits: "+str(maxPrecCtr)
print "Precision for Controllers(F and G) MIX: total number of bits: "+str(totPrecCtr)
print "Mixed uses "+str(100.0-(float(totPrecCtr)/maxPrecCtr)*100.0)+"% less than uniform\n\n"

print "Precision for Controllers(H and K) UNIFORM: Float"+str(maxValueBrd)+" Total number of bits: "+str(maxPrecBrd)
print "Precision for Controllers(H and K) MIX: total number of bits: "+str(totPrecBrd)
print "Mixed uses "+str(100.0-(float(totPrecBrd)/maxPrecBrd)*100.0)+"% less than uniform"

print "\n\n###COMPLETE###"
exit(0)




