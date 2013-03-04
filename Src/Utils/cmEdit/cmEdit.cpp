#include "Tools/Configuration/ConfigMap.h"
#include "Tools/Configuration/ConfigMapParser.h"

#include <iostream>

void usage(const std::string& arg0)
{
  std::cerr << "usage: " << arg0 << " [-l] filename | -q filename key | -d defaults filename" << std::endl;
}

int main(int argc, char** argv)
{
  if(argc < 2)
  {
    usage(argv[0]);
    return -1;
  }
  if(std::string(argv[1]) == "-l")
  {
    if(argc < 3)
    {
      usage(argv[0]);
      return -2;
    }
    ConfigMapLexer lexer(argv[2]);
    ConfigMapLexer::Token t(ConfigMapLexer::Token::CMT_EOF, 0, 0, "");
    while((t = lexer.nextToken()).type != ConfigMapLexer::Token::CMT_EOF)
    {
      std::cout << "[ " << ConfigMapLexer::Token::type2str(t.type) << " : " << t.line << "," << t.column << " \"" << t.value << "\"]" << std::endl;
    }
  }
  else if(std::string(argv[1]) == "-q")
  {
    if(argc < 4)
    {
      usage(argv[0]);
      return -3;
    }
    ConfigMap cm;
    int result = cm.read(argv[2], true);
    std::cout << cm[std::string(argv[3])] << std::endl
              << std::endl << "Result: " << result << std::endl;
  }
  else if (std::string(argv[1]) == "-d")
  {
    if (argc != 4)
    {
      usage(argv[0]);
      return -4;
    }
    ConfigMap defaults;
    int result = defaults.read(argv[2], true);
    if (result < 0)
    {
      return -4;
    }
    std::cout << "--defaults:" << std::endl;
    std::cout << defaults << std::endl;
    ConfigMap cm;
    result = cm.read(argv[3], true);
    if (result < 0)
    {
      return -4;
    }
    std::cout << "--map:" << std::endl;
    std::cout << cm << std::endl;
    defaults += cm;
    std::cout << "Map has " << defaults.length() << " entries." << std::endl;
    std::cout << defaults << std::endl;
  }
  else
  {
    ConfigMap cm;
    int result = cm.read(argv[1], true);
    std::cout << result << std::endl;
    std::cout << cm << std::endl;
  }
  return 0;
}
