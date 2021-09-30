import subprocess
import struct

MEM_ADDR = 0x40000000

def write(data,header):
    response = {"err":False,"errMsg":"","data":b''}

    if ("debug" in header) and (header["debug"]):
        response["errMsg"] = "Message received"
        return response     
    elif header["mode"] == "write":
        for i in range(0,len(data),2):
            addr = MEM_ADDR + data[i]
            cmd = ['monitor',format(addr),'0x' + '{:0>8x}'.format(data[i+1])]
            if ("print" in header) and (header["print"]):
                print("Command: ",cmd)
            result = subprocess.run(cmd,stdout=subprocess.PIPE)
            if result.returncode != 0:
                break
            else:
                tmp = result.stdout.decode('ascii').rstrip()
                if len(tmp) > 0:
                    response["data"] += struct.pack("<I",int(tmp,16))

    elif header["mode"] == "read":
        for i in range(0,len(data)):
            addr = MEM_ADDR + data[i]
            cmd = ['monitor',format(addr)]
            if ("print" in header) and (header["print"]):
                print("Command: ",cmd)
            result = subprocess.run(cmd,stdout=subprocess.PIPE)
            if result.returncode != 0:
                break
            else:
                tmp = result.stdout.decode('ascii').rstrip()
                if len(tmp) > 0:
                    response["data"] += struct.pack("<I",int(tmp,16))

    elif header["mode"] == "get scan data":
        if header["reset"]:
            cmd = ['./saveScanData','-rn',format(header["numSamples"]),'-t',format(2)]
        else:
            cmd = ['./saveScanData','-n',format(header["numSamples"]),'-t',format(2)]

        if ("print" in header) and (header["print"]):
            print("Command: ",cmd)
        result = subprocess.run(cmd,stdout=subprocess.PIPE)

        if result.returncode == 0:
            fid = open("saved-scan-data.bin","rb")
            response["data"] = fid.read()
            fid.close()
    
    
    if result.returncode != 0:
        response = {"err":True,"errMsg":"Bus error","data":b''}

    return response
        


    
        
