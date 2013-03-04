
class StateMachine
{
public:
  class CppCode
  {
  public:
    std::string file;
    unsigned int line;
    std::string code;
  };

  class Option
  {
  public:
    class State
    {
    public:
      std::string name;
      std::string file;
      unsigned int line;
      std::list<CppCode> decisionCode;
      std::list<CppCode> actionCode;
    };

    std::string name;
    std::string file;
    unsigned int line;
    std::list<CppCode> code;
    std::list<CppCode> commonDecisionCode;
    std::list<CppCode> commonActionCode;
    std::list<State> states;
    std::unordered_map<std::string, State*> statesByName;
  };

  class File
  {
  public:
    std::string name;
    std::list<CppCode> code;
    std::list<Option> options;
    std::unordered_map<std::string, Option*> optionsByName;
  };

  class Input
  {
  public:
    std::string type;
    std::string name;
    std::string file;
    unsigned int line;
  };

  class Output
  {
  public:
    std::string type;
    std::string name;
    std::string file;
    unsigned int line;
  };

  std::list<Input> input;
  std::unordered_map<std::string, Input*> inputByName;
  std::list<Output> output;
  std::unordered_map<std::string, Output*> outputByName;
  std::list<File> files;
};

