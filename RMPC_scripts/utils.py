import sys
import shlex
import subprocess
from decimal import *
import os

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

def takeBounds(index,lines):
	retList=[]
	if lines[index]=="\n":
		return
	i=index
	numBounds=len(lines)
	while i<numBounds and not "##" in lines[i]:
		if not lines[i]=="\n":
			bounds=[]
			values=lines[i].split(" ")
			for val in values:
				if val:
					bounds.append(tuple((str(val.strip()),"0")))
			i=i+1
			values=lines[i].split(" ")
			t=0
			for val in values:
				if val:
					tmp=list(bounds[t])
					tmp[1]=str(val.strip())
					bounds[t]=tuple(tmp)
					t=t+1
			retList.append(bounds)
		i=i+1
	return retList
	
def get_i_j_from_tuple(tupleVal):
	vals=str(tupleVal).split(",")
	i=vals[0].replace("(","")
	j=vals[1].replace(")","")
	return int(i),int(j)
	
def importKodiakBounds(filename):
	f=open("/home/roki/GIT/Kodiak/cmake-build-debug/examples/"+filename,"r")
	lines=f.readlines()
	X=[]
	for index,line in enumerate(lines):
		if not line=="\n":
			if "## Certainly:" in line:
				tmp=takeBounds(index+1,lines)
				if not tmp is None:
					X=X+tmp
				continue
			if "## Possibly:" in line:
				tmp=takeBounds(index+1,lines)
				if not tmp is None:
					X=X+tmp
				continue
			if "## Almost Certainly:" in line:
				tmp=takeBounds(index+1,lines)
				if not tmp is None:
					X=X+tmp
				continue
	return X

def checkBoundsControllers_i_j_(i,j,X,F,G,dimensions):
	finalStr=""
	#for index,i_bound in enumerate(X):
	index=0
	#print "i:"+str(i)+", j:"+str(j)
			
	finalStr=finalStr+"def U_"+str(i)+"_U_"+str(j)+"_"+str(index)+"("+encodeInputVector("X",dimensions)+"):Real={\n"
	finalStr=finalStr+"require("+encodeRangeInputVector(X)+")\n\n"
	finalStr=finalStr+"val U_"+str(i)+"_U_"+str(j)+"_"+str(index)+" = "

	tmp_i=""
	tmp_j=""

	for ind in range(0,dimensions):
		tmp_i=tmp_i+"("+F[i][ind]+")"+"*X"+str(ind)+"+"
		tmp_j=tmp_j+"("+F[j][ind]+")"+"*X"+str(ind)+"+"
	
	tmp_i=tmp_i+"("+G[i]+")"
	tmp_j=tmp_j+"("+G[j]+")"
	
	finalStr=finalStr+tmp_i+" - ("+tmp_j+")\n\n"	
	finalStr=finalStr+"U_"+str(i)+"_U_"+str(j)+"_"+str(index)+"\n"
	finalStr=finalStr+"\n}"
	finalStr=finalStr+"\n\n"
	
	return finalStr

#A is a vector 2X2
#X is a vector 2X1
#B is a scalar
#U is a scalar
#the result is X(i) t+1 where 'i' is the index in X (that is a vector)

def encodeInputVector(letter,dimensions):
	finalStr=""
	for index in range(0,dimensions):
		finalStr=finalStr+letter+str(index)+":Real, "
	finalStr=finalStr[:-2]
	return finalStr

def encodeRangeInputVector(X):
	finalStr=""
	for val in X:
		finalStr=finalStr+val+" && "
	finalStr=finalStr[:-4]
	return finalStr

def findMaxBoundValue(lines):
	#U_2_U_1_1,0.0015088578641414643,0.0019545455582414095,-0.7885675,-0.77197375
	maxValue=Decimal("0")
	for line in lines:
		valueCleanLB=line.split(",")[3]
		valueCleanUB=line.split(",")[4]
		#valueClean=valuesRaw.split(",")[0]
		if valueCleanLB.startswith("-"):
			valueCleanLB=valueCleanLB[1:]
		if valueCleanUB.startswith("-"):
			valueCleanUB=valueCleanUB[1:]
		if Decimal(valueCleanLB)>maxValue:
			maxValue=Decimal(valueCleanLB)
		if Decimal(valueCleanUB)>maxValue:
			maxValue=Decimal(valueCleanUB)
	return maxValue
	
def checkControllersChoiseError(filename,dimensions,NumControllers,deltaX,F,G,boundsX,HXK,divLimit,totalOpt):

	checkInput(F,G,HXK,NumControllers,dimensions)
	processes=[]

	for tupleVal in deltaX:
		
		i,j=get_i_j_from_tuple(tupleVal)
					
		print "Mistake in choosing the active Controller:"+str(tupleVal)
		
		#X=boundsX[i]+HXK[i]+deltaX[tupleVal]
		
		X=deltaX[tupleVal]
										
		encoding="import daisy.lang._\nimport Real._\nobject "+filename+" {\n\n"
		encoding=encoding+checkBoundsControllers_i_j_(i,j,X,F,G,dimensions)+"\n"
		encoding=encoding+"}"
		
		f = open("./inputFiles/"+filename+"_safe_bounds_"+str(i)+"_"+str(j)+".scala","w+")
		f.write(encoding)
		f.close()

		if os.path.exists("./output/"+filename+"_ctr_choise_from_"+str(i)+"_"+str(j)+".txt"):
			os.remove("./output/"+filename+"_ctr_choise_from_"+str(i)+"_"+str(j)+".txt")
	
		exe="./daisy --precision=Fixed64 --rangeMethod=smt --solver=dReal --subdiv --divLimit="+str(divLimit)+" --totalOpt="+str(totalOpt)+" --results-csv="+filename+"_ctr_choise_from_"+str(i)+"_"+str(j)+".txt ./inputFiles/"+filename+"_safe_bounds_"+str(i)+"_"+str(j)+".scala"
		
		exe=shlex.split(exe)
		
		p=subprocess.Popen(exe,shell=False)
			
		p.wait()
				
		#processes.append(p)

	for p in processes:
		p.wait()
		
	return

def getMaxErrorFromChoise(filename,deltaX):
	maxError={}
	for tupleVal in deltaX:
		i,j=get_i_j_from_tuple(tupleVal)
		f= open("./output/"+filename+"_ctr_choise_from_"+str(i)+"_"+str(j)+".txt","r")
		lines=f.readlines()
		maxError[str(tupleVal)]=findMaxBoundValue(lines[1:])
		f.close()
	
	#print maxError
	return maxError
