import subprocess
import sys
import random
import os

def onPprocessors(command,p) :
  if os.environ.has_key("OPENMP"):
    os.putenv("OMP_NUM_THREADS", "%d" %p)
    return command
  elif os.environ.has_key("CILK"):
    return command + " -cilk_set_worker_count " + `p`
  elif os.environ.has_key("MKLROOT"):
    return "export CILK_NWORKERS="+`p`+"; " + command
  return command

def shellGetOutput(str) :
  process = subprocess.Popen(str,shell=True,stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE)
  output, err = process.communicate()

  if (len(err) > 0):
      raise NameError(str+"\n"+output+err)
  return output

def stripFloat(val) :
  trunc = float(int(val*1000))/1000
  return str(trunc).rstrip('0')

def runSingle(runProgram, options, ifile, procs) :
  comString = "./"+runProgram+" "+options+" "+ifile
  if (procs > 0) :
    comString = onPprocessors(comString,procs)
  out = shellGetOutput(comString)
  #print(out)
  try:
    times = [float(str[str.index(' ')+1:]) for str in out.split('\n') if str.startswith("PBBS-time: ")]
    return times
  except (ValueError,IndexError):
    raise NameError(comString+"\n"+out)

def runTest(runProgram, checkProgram, dataDir, test, rounds, procs, noOutput) :
    random.seed()
    outFile="/tmp/ofile%d_%d" %(random.randint(0, 1000000), random.randint(0, 1000000))
    [weight, inputFileNames, runOptions, checkOptions] = test
    if type(inputFileNames) is str :
      inputFileNames = [inputFileNames]
    shortInputNames = " ".join(inputFileNames)
    if len(dataDir)>0:
      out = shellGetOutput("cd " + dataDir + "; make " + shortInputNames)
    longInputNames = " ".join(dataDir + "/" + name for name in inputFileNames)
    runOptions = runOptions + " -r " + `rounds`
    if (noOutput == 0) :
      runOptions = runOptions + " -o " + outFile
    times = runSingle(runProgram, runOptions, longInputNames, procs)
    if (noOutput == 0) :
      checkString = ("./" + checkProgram + " " + checkOptions + " "
                     + longInputNames + " " + outFile)
      checkOut = shellGetOutput(checkString)
      # Allow checker output comments. Comments are lines prefixed by '::'
      nonCommentLines = [s for s in checkOut.split('\n') if not s.startswith(':') and len(s)>0]
      if (len(nonCommentLines) > 0) :
        print("CheckOut:", checkOut)
        raise NameError(checkString+"\n"+checkOut)
      os.remove(outFile)
    ptimes = str([stripFloat(time)
                  for time in times])[1:-1]
    outputStr = ""
    if (len(runOptions) > 0) :
      outputStr = " : " + runOptions
    print(`weight` + " : " + shortInputNames + outputStr + " : "
          + ptimes)
    return [weight,times]

def averageTime(times) :
    return sum(times)/len(times)


def timeAll(name, runProgram, checkProgram, dataDir, tests, rounds, procs, noOutput,
            addToDatabase, problem) :
  totalTime = 0
  totalWeight = 0
  try:
    results = [runTest(runProgram, checkProgram, dataDir, test, rounds, procs,
                       noOutput)
               for test in tests]
    totalTimeMean = 0
    totalTimeMin = 0
    totalTimeMedian = 0
    totalWeight = 0
    j = 0
    for (weight,times) in results:
      l = len(times)
      if (l == 0):
        print("Warning, no timed results for", tests[j])
        continue
      times = sorted(times)
      totalTimeMean = totalTimeMean + weight*sum(times)/l
      totalTimeMin = totalTimeMin + weight*times[0]
      totalTimeMedian = totalTimeMedian + weight*times[(l-1)/2]
      totalWeight = totalWeight + weight
      j += 1
    print(name + " : " + `procs` +" : " +
          "weighted time, min=" + stripFloat(totalTimeMin/totalWeight) +
          " median=" + stripFloat(totalTimeMedian/totalWeight) +
          " mean=" + stripFloat(totalTimeMean/totalWeight))
    if (addToDatabase) :
      try:
        dbAddResult(problem=problem, program=runProgram, results=results, numProcs=procs, mean=totalTimeMean/totalWeight,
                    min=totalTimeMin/totalWeight, median=totalTimeMedian/totalWeight, tests=tests)
      except:
        print("Could not insert result in database. Error:", sys.exc_info()[0])
#        if (os.getlogin() == 'akyrola'):  raise
    return 0
  except NameError,v:
    x, = v
    print "TEST TERMINATED ABNORMALLY:\n["+x + "]"
    return 1
  except KeyboardInterrupt:
    return 1


def getOption(str) :
  a = sys.argv
  l = len(a)
  for i in range(1, l) :
    if (a[i] == str) :
      return True
  return False

def getArg(str, default) :
  a = sys.argv
  l = len(a)
  for i in range(1, l) :
    if (a[i] == str and  (i+1 != l)) :
        return sys.argv[i+1]
  return default

def getArgs() :
  noOutput = getOption("-x")
  addToDatabase = getOption("-d")
  processors = int(getArg("-p", 0))
  rounds = int(getArg("-r", 1))
  return (noOutput, rounds, addToDatabase, processors)

def timeAllArgs(runProgram, problem, checkProgram, dataDir, tests) :
    (noOutput, rounds, addToDatabase, procs) = getArgs()
    name = os.path.basename(os.getcwd())
    timeAll(name, runProgram, checkProgram, dataDir, tests, rounds, procs, noOutput, addToDatabase, problem)

#
# Database insertions
# - akyrola@cs.cmu.edu

import os

def dbInitConnection():
    import MySQLdb
    global cursor
    # TODO: move to a config file
    dbconn = MySQLdb.connect (host = "multi6.aladdin.cs.cmu.edu",
                                                            user = "pbbs",
                                                            passwd = "pbbspasshuuhaa",
                                                            db = "pbbsweb")

    cursor = dbconn.cursor ()
    dbconn.autocommit(1)



def dbAddResult(problem, program, results, numProcs, mean, min, median, tests):
    dbInitConnection()
    contentHash = computeContentHash(tests)
    program = shellGetOutput("pwd").split('/')[-1].replace('\r','').replace('\n', '') + '/' + program
    problemId = dbGetProblemId(problem, contentHash)
    programId = dbGetProgramId(program, problemId)
    hostId = getHostId()


    #username = os.getlogin()
    # getlogin does not work with some terminals (see various posts on web)
    # guyb replaced with the following
    username = os.getenv('USER')
    if (numProcs == 0): numProcs = detectCPUs()
    # Insert run into db
    cursor.execute(""" insert into pbbs_runs (problem_id,program_id,numprocs,mean_time,min_time,median_time,username,host_id) values(
                                                %s,      %s,          %s,      %s,       %s,       %s,       %s,      %s)
                       """, (problemId, programId, numProcs, mean, min, median, username, hostId))
    cursor.execute(" select last_insert_id()")
    runId = cursor.fetchone()[0]

    for i in range(0, len(results)):
        (weight, times) = results[i]
        test = tests[i]
        [weight,inputFileNames,runOptions,checkOptions] = test
        if type(inputFileNames) is list :
          inputFileNames = "+".join(inputFileNames)
        for time in times:
            cursor.execute(""" insert into pbbs_subruns(run_id, inputfile, time, weight, params, check_params) values(
                                                       %s,          %s      , %s ,   %s,       %s,     %s) """,
                                                        (runId, inputFileNames, time, weight, runOptions, checkOptions))


def computeContentHash(tests):
    hash = ""
    for test in tests:
        [weight,inputFileNames,runOptions,checkOptions] = test
        if type(inputFileNames) is list :
          inputFileNames = "+".join(inputFileNames)
        hash += ";%f%s%s%s" %(weight,inputFileNames.strip(), runOptions.strip(),checkOptions.strip())
    hash = hash.replace(' ', '_')
    return hash

def dbGetProblemId(probname, contentHash):
    cursor.execute("select id from pbbs_problems where name=%s and content_hash=%s", (probname, contentHash))
    row = cursor.fetchone()
    if row == None:
        # Insert into db
        cursor.execute( "insert into pbbs_problems (name,content_hash) values(%s,%s) ", (probname, contentHash))
        cursor.execute(" select last_insert_id()")
        row = cursor.fetchone()
    return row[0]

def dbGetProgramId(progname, problemId):
    cursor.execute("select id from pbbs_programs where name=%s and problem_id=%s", (progname, problemId))
    row = cursor.fetchone()
    if row == None:
        # Insert into db
        cursor.execute( "insert into pbbs_programs (problem_id, name) values(%s, %s) ", (problemId, progname))
        cursor.execute(" select last_insert_id()")
        row = cursor.fetchone()
    return row[0]

import platform
def getHostId():
    (procmodel, mhz) = detectCPUModel()
    numprocs = detectCPUs()

    (sysname, nodename, release, version, machine) = os.uname()

    if (os.environ.has_key("OPENMP")):
       nodename = nodename + "[OPENMP]"

    cursor.execute("select id from pbbs_hosts where hostname=%s and procmodel=%s and version=%s and numprocs=%s", (nodename, procmodel, version, numprocs))
    row = cursor.fetchone()
    if row == None:
        cursor.execute(""" insert into pbbs_hosts(hostname,sysname,releasen,version,machine,numprocs,procmodel,mhz) values
                                                  (%s,      %s,        %s,   %s,    %s,    %s,           %s,  %s) """,
                                                    (nodename, sysname, release, version, machine, numprocs, procmodel, mhz))
        cursor.execute(" select last_insert_id()")
        row = cursor.fetchone()
    return row[0]

def detectCPUModel():
    mhz = 0
    model = platform.processor()
    try:
        if (platform.system() == "Darwin"):
            model = shellGetOutput("system_profiler SPHardwareDataType |grep 'Processor Name'")
            mhz = shellGetOutput("system_profiler SPHardwareDataType |grep 'Processor Speed'")
        else:
            model = shellGetOutput('grep "model name" /proc/cpuinfo').split('\n')[0]
            mhz = shellGetOutput('grep "cpu MHz" /proc/cpuinfo').split('\n')[0]
        model = model.split(':')[-1].strip()
        mhz = mhz.split(':')[-1].strip()
    except:
        # Could not get processor model
        print("Could not determine CPU model", sys.exc_info()[0])
    return (model, mhz)

def detectCPUs():
    """
     Detects the number of CPUs on a system. Cribbed from pp.
     """
    # Linux, Unix and MacOS:
    if hasattr(os, "sysconf"):
       if os.sysconf_names.has_key("SC_NPROCESSORS_ONLN"):
           # Linux & Unix:
           ncpus = os.sysconf("SC_NPROCESSORS_ONLN")
           if isinstance(ncpus, int) and ncpus > 0:
               return ncpus
       else: # OSX:
           return int(os.popen2("sysctl -n hw.ncpu")[1].read())
    # Windows:
    if os.environ.has_key("NUMBER_OF_PROCESSORS"):
           ncpus = int(os.environ["NUMBER_OF_PROCESSORS"]);
           if ncpus > 0:
               return ncpus
    return 1 # Default
