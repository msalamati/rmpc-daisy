import sys
import shlex
import subprocess
from decimal import *
import os
from utils import *
	
def defineControllerU_i(i,dimensions,X,F,G,delta="0"):
	finalStr=""
	index=0

	finalStr=finalStr+"def U"+str(i)+"_"+str(index)+"("+encodeInputVector("X",dimensions)+"):Real={\n"
	finalStr=finalStr+"require("+encodeRangeInputVector(X)+")\n\n"

	tmp=""
	for j in range(0,dimensions):
		tmp=tmp+"("+F[j]+")*X"+str(j)+"+"
	
	tmp=tmp+"("+G+")"

	finalStr=finalStr+tmp+"\n\n"		
	finalStr=finalStr+"\n}"
	
	if delta!="0":
		finalStr=finalStr+"ensuring (res => res +/- "+delta+")\n\n"
	else:
		finalStr=finalStr+"\n\n"
	
	return finalStr
	
def defineLine(U_id,Line_id,boundlineEquation,dimensions,X,delta):
	finalStr=""
	index=0

	finalStr=finalStr+"def U_"+str(U_id)+"_Line_"+str(Line_id)+"("+encodeInputVector("X",dimensions)+"):Real={\n"
	finalStr=finalStr+"require("+encodeRangeInputVector(X)+")\n\n"
	
	lineEquation=boundlineEquation.replace("<=","-")
	
	finalStr=finalStr+lineEquation+"\n\n"		
	finalStr=finalStr+"\n}"
	
	if delta!="0":
		finalStr=finalStr+"ensuring (res => res +/- "+delta+")\n\n"
	else:
		finalStr=finalStr+"\n\n"
	
	return finalStr

def findMax(lines):
	maxValue=Decimal("0")
	for line in lines:
		valueClean=line.split(",")[1]
		#valueClean=valuesRaw.split(",")[0]
		if Decimal(valueClean)>maxValue:
			maxValue=Decimal(valueClean)
	return maxValue

def PrecisionTuningForFXG(filename,dimensions,NumControllers,X,F,G,precision="0",delta="0"):
	
	processes=[]	
	
	for i in range(0,NumControllers):
		
		print "Quantization of Controller_"+str(i)

		X_final=X[i] #we do not need to encode the exact bounds in this case
		
		encoding_i="import daisy.lang._\nimport Real._\nobject "+filename+str(i)+" {\n\n"
		encoding_i=encoding_i+defineControllerU_i(i,dimensions,X_final,F[i],G[i],delta)+"\n"
		encoding_i=encoding_i+"}"
		
		f= open("./inputFiles/"+filename+"_controllers_"+str(i)+".scala","w+")
		f.write(encoding_i)
		f.close()
		
		if os.path.exists("./output/tmp_"+filename+"_controllers_"+str(i)+".txt"):
			os.remove("./output/tmp_"+filename+"_controllers_"+str(i)+".txt")
		
		if precision!="0":
			exe="./daisy --precision="+precision+" --results-csv=tmp_"+filename+"_controllers_"+str(i)+".txt ./inputFiles/"+filename+"_controllers_"+str(i)+".scala"
		else:
			lineUNI="./daisy --mixed-fixedpoint --choosePrecision=fixed --precision=Fixed64 --results-csv=UNI_"+filename+"_controllers_"+str(i)+".txt ./inputFiles/"+filename+"_controllers_"+str(i)+".scala"
			lineMIX="./daisy --mixed-tuning --precision=Fixed64 --results-csv=MIX_"+filename+"_controllers_"+str(i)+".txt ./inputFiles/"+filename+"_controllers_"+str(i)+".scala"
		
		exeUNI=shlex.split(lineUNI)
		traceUNI=open("./output/UNI_Trace_"+filename+"_controllers_"+str(i)+".txt","w")
		pUNI=subprocess.Popen(exeUNI,shell=False,stdout=traceUNI)
		
		exeMIX=shlex.split(lineMIX)
		traceMIX=open("./output/MIX_Trace_"+filename+"_controllers_"+str(i)+".txt","w")
		pMIX=subprocess.Popen(exeMIX,shell=False,stdout=traceMIX)
		
		pMIX.wait()
		pUNI.wait()
		
		#processes.append(pUNI)
		#processes.append(pMIX)
		
	for p in processes:
		p.wait()
		
def PrecisionTuningForHXK(filename,numControllers,dimensions,HXK,precision="0",delta="0"):
	processes=[]
	for U_id,listLines in HXK.iteritems():
		for Line_id,tupleLineBounds in enumerate(listLines):	
									
			line=str(tupleLineBounds[0])
			bounds= tupleLineBounds[1]
			encoding_i="import daisy.lang._\nimport Real._\nobject "+filename+"_U_"+str(U_id)+"_Line_"+str(Line_id)+" {\n\n"
			encoding_i=encoding_i+defineLine(U_id,Line_id,line,dimensions,bounds,delta)+"\n"
			encoding_i=encoding_i+"}"
			
			f= open("./inputFiles/"+filename+"_Line_U_"+str(U_id)+"_"+str(Line_id)+".scala","w+")
			f.write(encoding_i)
			f.close()
			
			if os.path.exists("./output/tmp_"+filename+"_Line_U_"+str(U_id)+"_"+str(Line_id)+".txxt"):
				os.remove("./output/tmp_"+filename++"_Line_U_"+str(U_id)+"_"+str(Line_id)+".txt")
			
			lineUNI="./daisy --mixed-fixedpoint --choosePrecision=fixed --precision=Fixed64 --rangeMethod=interval --results-csv=UNI_"+filename+"_Line_U_"+str(U_id)+"_"+str(Line_id)+".txt ./inputFiles/"+filename+"_Line_U_"+str(U_id)+"_"+str(Line_id)+".scala"
			lineMIX="./daisy --mixed-tuning --precision=Fixed64 --rangeMethod=interval --results-csv=MIX_"+filename+"_Line_U_"+str(U_id)+"_"+str(Line_id)+".txt ./inputFiles/"+filename+"_Line_U_"+str(U_id)+"_"+str(Line_id)+".scala"
			
			#print lineUNI
			#print lineMIX
			
			exeUNI=shlex.split(lineUNI)
			traceUNI=open("./output/UNI_Trace_"+filename+"_Line_U_"+str(U_id)+"_"+str(Line_id)+".txt","w")
			pUNI=subprocess.Popen(exeUNI,shell=False,stdout=traceUNI)
			

						
			exeMIX=shlex.split(lineMIX)
			traceMIX=open("./output/MIX_Trace_"+filename+"_Line_U_"+str(U_id)+"_"+str(Line_id)+".txt","w")
			pMIX=subprocess.Popen(exeMIX,shell=False,stdout=traceMIX)
			
			pUNI.wait()
			pMIX.wait()

			#processes.append(pUNI)
			#processes.append(pMIX)
			
			print "Quantization of bound: Line U_"+str(U_id)+"_"+str(Line_id)
						
	for p in processes:
		p.wait()

def getAbsErrorAfterFailure(line):
	if "Absolute error:" in line:
		values=line.split("Absolute error:")
		val=(values[1].strip()).split(".")[0]
		try:
			float(val)
			return val
		except:
			print "Problem during parsing of the absolute error"
			exit(0)
			
def readMaxUniformPrecisionController(filename,NumControllers):
	precision=""
	maxPrecision=0
	warningExists=False
	reachMaxPrec=False
	failure=False
	maxErrorFailure=""
	for i in range(0,NumControllers):
		f= open("./output/UNI_Trace_"+filename+"_controllers_"+str(i)+".txt")
		lines=f.readlines()
		for line in lines:
			if "trying precision Fixed" in line:
				val=(line.split("trying precision Fixed")[1])
				if int(val)>maxPrecision:
					maxPrecision=int(val)
			if "Warning" in line:
				warningExists=True
			if "Highest available precision Fixed64 is not sufficient" in line:
				reachMaxPrec=True
			if reachMaxPrec:
				maxErrorFailure=getAbsErrorAfterFailure(line)
		if warningExists and reachMaxPrec:
			failure=True
		f.close()
		#print maxPrecision
	return failure,maxErrorFailure,maxPrecision,maxPrecision*NumControllers*6
	
def readMixedPrecisionConfigController(filename,NumControllers):
	precision=""
	totNumber=0
	for i in range(0,NumControllers):
		f= open("./output/"+filename+str(i)+".cpp")
		lines=f.readlines()
		for line in lines:
			#print line
			if ";" in line and "<" in line and ">" in line:
				vals=line.split("<")[1]
				vals=vals.split(">")[0]
				val=vals.split(",")[0]
				totNumber=totNumber+int(val)
		#print totNumber
		f.close()
	return totNumber
	
def readMaxUniformPrecisionBorders(filename,NumControllers,HXK):
	precision=""
	maxPrecision=0
	totEdges=0
	warningExists=False
	reachMaxPrec=False
	failure=False
	maxErrorFailure=""
	for i in range(0,NumControllers):
		for j in range(0,len(HXK[i])):
			totEdges=totEdges+1
			#UNI_Trace_cruise_Line_U_3_4.scala
			try:
				f= open("./output/UNI_Trace_"+filename+"_Line_U_"+str(i)+"_"+str(j)+".txt")
				lines=f.readlines()
				for line in lines:
					#print line
					if "trying precision Fixed" in line:
						val=(line.split("trying precision Fixed")[1])
						if int(val)>maxPrecision:
							maxPrecision=int(val)
					if "Warning" in line:
						warningExists=True
					if "Highest available precision Fixed64 is not sufficient" in line:
						reachMaxPrec=True
					if reachMaxPrec:
						maxErrorFailure=getAbsErrorAfterFailure(line)
				if warningExists and reachMaxPrec:
					failure=True
				f.close()
			except:
				print "Error: ./output/UNI_Trace_"+filename+"_Line_U_"+str(i)+"_"+str(j)+".txt does not exist!!"
				exit(0)
	return failure,maxErrorFailure,maxPrecision,maxPrecision*6*totEdges
	
def readMixedPrecisionConfigBorders(filename,NumControllers,HXK):
	precision=""
	totNumber=0
	for i in range(0,NumControllers):
		for j in range(0,len(HXK[i])):
			#cruise_U_3_Line_0.cpp
			try:
				f= open("./output/"+filename+"_U_"+str(i)+"_Line_"+str(j)+".cpp")
				lines=f.readlines()
				for line in lines:
					#print line
					if ";" in line and "<" in line and ">" in line:
						vals=line.split("<")[1]
						vals=vals.split(">")[0]
						val=vals.split(",")[0]
						totNumber=totNumber+int(val)
				#print totNumber
				f.close()
			except:
				print "Error: ./output/"+filename+"_U_"+str(i)+"_Line_"+str(j)+".cpp does not exist!"
				exit(0)
	return totNumber
