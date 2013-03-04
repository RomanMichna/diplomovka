/**
* @file StateMachineCompiler.cpp
* A compiler for state machine description files used by the StateMachineBehaviorEngine module
* @author Colin Graf
*/

#include <cassert>
#include <cctype>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>
#include <cstring>
#include <list>
#include <string>
#ifndef WIN32
#include <tr1/unordered_map>
namespace std { using namespace tr1; }
#else
#include <unordered_map>
#endif

#include "Error.h"
#include "StateMachine.h"
#include "Parser.h"
#include "Generator.h"

void usage()
{
  printf("usage:\n StateMachineCompiler [ -I <input-dir> ] <input1> [ <input1> ... ] -o <output> [ -p <prototype-output> ]\n\n");
  exit(EXIT_FAILURE);
}

int main(int argc, char* argv[])
{
  std::list<std::string> inputFiles;
  std::string outputFile;
  std::string prototypeFile;
  std::string inputDir;

  // parse arguments
  for(int i = 1; i < argc; ++i)
  {
    if(!strcmp(argv[i], "-o"))
    {
      ++i;
      if(i >= argc && !outputFile.empty())
        usage();
      outputFile = argv[i];
    }
    else if(!strcmp(argv[i], "-p"))
    {
      ++i;
      if(i >= argc && !prototypeFile.empty())
        usage();
      prototypeFile = argv[i];
    }
    else if(!strcmp(argv[i], "-I"))
    {
      ++i;
      if(i >= argc && !inputDir.empty())
        usage();
      inputDir = argv[i];
      if(!inputDir.empty() && !strchr("/\\", inputDir.at(inputDir.length() - 1)))
        inputDir += "/";
    }
    else if(argv[i][0] == '-')
      usage();
    else
      inputFiles.push_back(inputDir + argv[i]);
  }
  if(inputFiles.empty() || outputFile.empty())
    usage();

  // read input files
  StateMachine stateMachine;
  {
    Parser parser;
    for(std::list<std::string>::const_iterator i = inputFiles.begin(), end = inputFiles.end(); i != end; ++i)
      if(!parser.parse(*i, stateMachine))
        return EXIT_FAILURE;
  }

  // generate output
  Generator generator;
  if(!generator.generate(stateMachine, outputFile))
    return EXIT_FAILURE;

  // generate protype output
  if(!prototypeFile.empty())
    if(!generator.generatePrototype(stateMachine, prototypeFile))
      return EXIT_FAILURE;

  return EXIT_SUCCESS;
}
