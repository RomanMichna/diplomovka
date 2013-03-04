
#include <cstdio>

#include "Representations/Configuration/ColorTable64.h"
#include "Tools/Streams/InStreams.h"

int main(int argc, char** argv)
{
  if(argc != 2)
  {
    printf("usage: %s filename\n", argv[0]);
    return -1;
  }

  ColorTable64 ct;
  std::string filename(argv[1]);
  InBinaryFile stream(filename);
  if(!stream.exists())
  {
    fprintf(stderr, "Cannot open \"%s\".\n", filename.c_str());
    return 1;
  }
  stream >> ct;
  printf("%s\t%s\n", ct.hash().c_str(), filename.c_str());
  return 0;
}
