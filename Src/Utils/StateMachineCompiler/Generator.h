
class Generator
{
public:

  Generator() : useLineDirective(true), fp(0) {}

  ~Generator()
  {
    if(fp)
      fclose(fp);
  }

  bool generate(const StateMachine& stateMachine, const std::string& file)
  {
    if(!openFile(file))
      return false;

write("\n");
write("#include \"StateMachineBehavior.h\"\n");
write("#include \"StateMachineBehaviorEngine.h\"\n");
write("\n");

// add global code from options files
write("#define __DISABLE_STATE_MACHINE_PROTOTYPES\n");
write("\n");
for(std::list<StateMachine::File>::const_iterator j = stateMachine.files.begin(), end = stateMachine.files.end(); j != end; ++j)
{
  const StateMachine::File& file = *j;
  unsigned int line = 0;
  for(std::list<StateMachine::CppCode>::const_iterator i = file.code.begin(), end = file.code.end(); i != end; ++i)
  {
    const StateMachine::CppCode& cppCode = *i;
    if(useLineDirective)
    {
      if(line == 0)
        write(std::string("#line ") + itoa(cppCode.line) + " \"" + cppCode.file + "\"\n");
      else if(cppCode.line != line + 1)
        write(std::string("#line ") + itoa(cppCode.line) + "\n");
    }
    line = cppCode.line;
    write(cppCode.code + "\n");
  }
write("\n");
}

write("class StateMachineBehaviorData\n");
write("{\n");
write("public:\n");
write("\n");
write("  StateMachineBehaviorData() : __rootOption(0), __step(1), __time(0), __depth(0), __deciding(false) {}\n");
write("private:\n");

// class prototype for each option
for(std::list<StateMachine::File>::const_iterator j = stateMachine.files.begin(), end = stateMachine.files.end(); j != end; ++j)
  for(std::list<StateMachine::Option>::const_iterator i = j->options.begin(), end = j->options.end(); i != end; ++i)
    write("class __" + i->name + "Option;\n");
write("\n");

// class for using "state" as datatype
write("class state\n");
write("{\n");
write("public:\n");
write("  state(unsigned int s = 0) : s(s) {}\n");
write("  operator unsigned int() const {return s;}\n");
write("private:\n");
write("  unsigned int s;\n");
write("};\n");
write("\n");

//
write("class __InputAndOutput\n");
write("{\n");
write("public:\n");
for(std::list<StateMachine::Input>::const_iterator i = stateMachine.input.begin(), end = stateMachine.input.end(); i != end; ++i)
  write("const " + i->type + "* " + i->name + ";\n");
for(std::list<StateMachine::Output>::const_iterator i = stateMachine.output.begin(), end = stateMachine.output.end(); i != end; ++i)
  write(i->type + "* " + i->name + ";\n");
write("};\n");
write("\n");

// baseclass for all options
write("class __Option\n");
write("{\n");
write("public:\n");
write("  /**\n");
write("  * Default constructor. Leaves all option references uninitialized.\n");
write("  */\n");
write("  __Option() :\n");
for(std::list<StateMachine::Input>::const_iterator i = stateMachine.input.begin(), end = stateMachine.input.end(); i != end; ++i)
  write(i->name + "(*__inputAndOutput->" + i->name + "),\n");
for(std::list<StateMachine::Output>::const_iterator i = stateMachine.output.begin(), end = stateMachine.output.end(); i != end; ++i)
  write(i->name + "(*__inputAndOutput->" + i->name + "),\n");
for(std::list<StateMachine::File>::const_iterator j = stateMachine.files.begin(), end = stateMachine.files.end(); j != end; ++j)
  for(std::list<StateMachine::Option>::const_iterator i = j->options.begin(), end = j->options.end(); i != end; ++i)
    write(i->name + "(" + i->name + "),\n");
write("    optionTime(0), stateTime(0), __data(0), __state(0), __lastStep(0), __lastDepth(0), __optionEnterTime(0), __stateEnterTime(0) {}\n");
write("\n");
write("  /**\n");
write("  * Constructor for initializing option references only.\n");
write("  */\n");
write("  __Option(StateMachineBehaviorData& __data) :\n");
for(std::list<StateMachine::Input>::const_iterator i = stateMachine.input.begin(), end = stateMachine.input.end(); i != end; ++i)
  write(i->name + "(" + i->name + "),\n");
for(std::list<StateMachine::Output>::const_iterator i = stateMachine.output.begin(), end = stateMachine.output.end(); i != end; ++i)
  write(i->name + "(" + i->name + "),\n");
for(std::list<StateMachine::File>::const_iterator j = stateMachine.files.begin(), end = stateMachine.files.end(); j != end; ++j)
  for(std::list<StateMachine::Option>::const_iterator i = j->options.begin(), end = j->options.end(); i != end; ++i)
    write(i->name + "(__data." + i->name + "),\n");
write("    __data(0) {}\n");
write("\n");
write("  /**\n");
write("  * Sets the option id and copys all option references from another option.\n");
write("  */\n");
write("  void __init(unsigned int __id, StateMachineBehaviorData& __data, __Option& optionReferences)\n");
write("  {\n");
write("    this->__id = __id;\n");
write("    this->__data = &__data;\n");
write("    for(void** i = &__optionsBegin + 1, ** end = &__optionsEnd, **j = &optionReferences.__optionsBegin + 1; i < end;)\n");
write("      *(i++) = *(j++);\n");
write("  }\n");
write("\n");
write("  /**\n");
write("  * Options can be activated by calling them like a function\n");
write("  */\n");
write("  unsigned int operator() ()\n");
write("  {\n");
write("    if(__lastStep == __data->__step)\n");
write("      return __state;\n");
write("    if(__data->__deciding)\n");
write("      return __lastStep + 1 != __data->__step ? 0 : __state;\n");
write("    if(__lastStep + 1 != __data->__step)\n");
write("    { // option was not active in previous frame => reset\n");
write("      __optionEnterTime = __data->__time;\n");
write("      __stateEnterTime = __data->__time;\n");
write("      optionTime = 0;\n");
write("      stateTime = 0;\n");
write("      __state = 0;\n");
write("    }\n");
write("    else\n");
write("    {\n");
write("      optionTime = __data->__time - __optionEnterTime;\n");
write("      stateTime = __data->__time - __stateEnterTime;\n");
write("    }\n");
write("    time = __data->__time;\n");
write("    __lastStep = __data->__step;\n");
write("    __lastDepth = __data->__depth;\n");
write("    ++__data->__depth;\n");
write("    __data->__activeOptions.push_back(this);\n");
write("\n");
write("    __data->__deciding = true;\n");
write("    unsigned int newState = __decision();\n");
write("    __data->__deciding = false;\n");
write("    if(newState != __state)\n");
write("    {\n");
write("      __stateEnterTime = __data->__time;\n");
write("      stateTime = 0;\n");
write("      __state = newState;\n");
write("    }\n");
write("    newState = __action();\n");
write("    if(newState != __state)\n");
write("    {\n");
write("      __stateEnterTime = __data->__time;\n");
write("      stateTime = 0;\n");
write("      __state = newState;\n");
write("    }\n");
write("    --__data->__depth;\n");
write("    return __state;\n");
write("  }\n");
write("\n");
write("protected:\n");
write("\n");
write("  // input representations\n");
for(std::list<StateMachine::Input>::const_iterator i = stateMachine.input.begin(), end = stateMachine.input.end(); i != end; ++i)
  write(std::string("const ") + i->type + "& " + i->name + ";\n");
write("\n");
write("  // output representations\n");
for(std::list<StateMachine::Output>::const_iterator i = stateMachine.output.begin(), end = stateMachine.output.end(); i != end; ++i)
  write(i->type + "& " + i->name + ";\n");
write("\n");
write("  // options\n");
write("  void* __optionsBegin;\n");
for(std::list<StateMachine::File>::const_iterator j = stateMachine.files.begin(), end = stateMachine.files.end(); j != end; ++j)
  for(std::list<StateMachine::Option>::const_iterator i = j->options.begin(), end = j->options.end(); i != end; ++i)
    write(std::string("__") + i->name + "Option& " + i->name + ";\n");
write("  void* __optionsEnd;\n");
write("\n");
write("  // special attributes\n");
write("  unsigned int time;\n");
write("  unsigned int optionTime;\n");
write("  unsigned int stateTime;\n");
write("\n");
write("private:\n");
write("  virtual unsigned int __decision() {return __state;};\n");
write("  virtual unsigned int __action() {return __state;};\n");
write("\n");
write("  StateMachineBehaviorData* __data;\n");
write("protected:\n");
write("  unsigned int __state;\n");
write("private:\n");
write("  unsigned int __id;\n");
write("  unsigned int __lastStep;\n");
write("  unsigned int __lastDepth;\n");
write("  unsigned int __optionEnterTime;\n");
write("  unsigned int __stateEnterTime;\n");
write("  unsigned int __lastAttribute;\n");
write("\n");
write("  static __InputAndOutput* __inputAndOutput;\n");
write("\n");
write("  friend class StateMachineBehavior;\n");
write("};\n");
write("\n");

// generate class for each option
for(std::list<StateMachine::File>::const_iterator j = stateMachine.files.begin(), end = stateMachine.files.end(); j != end; ++j)
  for(std::list<StateMachine::Option>::const_iterator i = j->options.begin(), end = j->options.end(); i != end; ++i)
  {
    const StateMachine::Option& option = *i;
    write("class __" + option.name + "Option : public __Option\n");
    write("{\n");
    write("public:\n");
    write("  enum __State\n");
    write("  {\n");
    for(std::list<StateMachine::Option::State>::const_iterator i = option.states.begin(), end = option.states.end(); i != end; ++i)
      write(i->name + ",\n");
    write("  };\n");
    write("\n");
    write("private:\n");
    write("  virtual unsigned int __decision();\n");
    write("  virtual unsigned int __action();\n");
    write("\n");
    unsigned int line = 0;
    for(std::list<StateMachine::CppCode>::const_iterator i = option.code.begin(), end = option.code.end(); i != end; ++i)
    {
      const StateMachine::CppCode& cppCode = *i;
      if(useLineDirective)
      {
        if(line == 0)
          write(std::string("#line ") + itoa(cppCode.line) + " \"" + cppCode.file + "\"\n");
        else if(cppCode.line != line + 1)
          write(std::string("#line ") + itoa(cppCode.line) + "\n");
      }
      line = cppCode.line;
      write(cppCode.code + "\n");
    }
    write("\n");
    write("  friend StateMachineBehaviorData::StateMachineBehaviorData();\n");
    write("};\n");
    write("\n");
  }

// instance of each option
for(std::list<StateMachine::File>::const_iterator j = stateMachine.files.begin(), end = stateMachine.files.end(); j != end; ++j)
  for(std::list<StateMachine::Option>::const_iterator i = j->options.begin(), end = j->options.end(); i != end; ++i)
    write("__" + i->name + "Option " + i->name + ";\n");
write("\n");

// further state information
write("  __Option* __rootOption;\n");
write("  unsigned int __step;\n");
write("  unsigned int __time;\n");
write("  unsigned int __depth;\n");
write("  bool __deciding;\n");
write("  std::vector<__Option*> __activeOptions;\n");
write("\n");

write("  __Option* __findOption(const char* agent)\n");
write("  {\n");
for(std::list<StateMachine::File>::const_iterator j = stateMachine.files.begin(), end = stateMachine.files.end(); j != end; ++j)
  for(std::list<StateMachine::Option>::const_iterator i = j->options.begin(), end = j->options.end(); i != end; ++i)
    write("if(!strcmp(agent, \"" + i->name + "\")) return &" + i->name + ";\n");
write("    return 0;\n");
write("  }\n");
write("\n");
write("  friend class StateMachineBehavior;\n");
write("};\n");
write("\n");

write("StateMachineBehaviorData::__InputAndOutput* StateMachineBehaviorData::__Option::__inputAndOutput = 0;\n");
write("\n");
write("StateMachineBehavior::StateMachineBehavior() : data(0) {}\n");
write("\n");
write("StateMachineBehavior::~StateMachineBehavior()\n");
write("{\n");
write("  if(data)\n");
write("    delete data;\n");
write("}\n");
write("\n");
write("bool StateMachineBehavior::init(StateMachineBehaviorEngine& engine, const std::string& agent)\n");
write("{\n");
write("  if(data)\n");
write("    delete data;\n");
write("  StateMachineBehaviorData::__InputAndOutput __inputAndOutput;\n");
for(std::list<StateMachine::Input>::const_iterator i = stateMachine.input.begin(), end = stateMachine.input.end(); i != end; ++i)
  write("__inputAndOutput." + i->name + " = &engine." + i->name + ";\n");
for(std::list<StateMachine::Output>::const_iterator i = stateMachine.output.begin(), end = stateMachine.output.end(); i != end; ++i)
  write("__inputAndOutput." + i->name + " = &engine." + i->name + ";\n");
write("  lock();\n");
write("  StateMachineBehaviorData::__Option::__inputAndOutput = &__inputAndOutput;\n");
write("  data = new StateMachineBehaviorData();\n");
write("  StateMachineBehaviorData::__Option::__inputAndOutput = 0;\n");
write("  unlock();\n");
write("  data->__rootOption = data->__findOption(agent.c_str());\n");
write("  if(!data->__rootOption)\n");
write("    return false;\n");
write("\n");
write("  StateMachineBehaviorData::__Option optionReferences(*data);\n");
write("\n");
int optionIndex = 0;
for(std::list<StateMachine::File>::const_iterator j = stateMachine.files.begin(), end = stateMachine.files.end(); j != end; ++j)
  for(std::list<StateMachine::Option>::const_iterator i = j->options.begin(), end = j->options.end(); i != end; ++i, ++optionIndex)
    write("data->" + i->name + ".__init(" + itoa(optionIndex) + ", *data, optionReferences);\n");
write("    return true;\n");
write("}\n");
write("\n");
write("void StateMachineBehavior::update(unsigned int time)\n");
write("{\n");
write("  if(!data->__rootOption)\n");
write("    return;\n");
write("  ++data->__step;\n");
write("  data->__time = time;\n");
write("  data->__activeOptions.resize(0);\n");
write("  (*data->__rootOption)();\n");
write("}\n");
write("\n");
write("void StateMachineBehavior::getOptions(std::vector<OptionInfo>& options)\n");
write("{\n");
for(std::list<StateMachine::File>::const_iterator j = stateMachine.files.begin(), end = stateMachine.files.end(); j != end; ++j)
  for(std::list<StateMachine::Option>::const_iterator i = j->options.begin(), end = j->options.end(); i != end; ++i)
  {
    const StateMachine::Option& option = *i;
    write("{\n");
    write("OptionInfo info;\n");
    write("info.name = \"" + option.name + "\";\n");
    write("info.states.resize(0);\n");
    write(std::string("    info.states.reserve(") + itoa(option.states.size()) + ");\n");
    if(option.states.size() > 0)
    {
      write("const char* stateNames[] = {\n");
      for(std::list<StateMachine::Option::State>::const_iterator i = option.states.begin(), end = option.states.end(); i != end; ++i)
        write("\"" + i->name + "\",\n");
      write("};\n");
      write(std::string("for(int i = 0; i < ") + buf + "; ++i)\n");
      write("info.states.push_back(stateNames[i]);\n");
    }
    write("options.push_back(info);\n");
    write("}\n");
  }
write("}\n");
write("\n");
write("void StateMachineBehavior::getActiveOptions(std::vector<ActiveOptionInfo>& options)\n");
write("{\n");
write("  ActiveOptionInfo info;\n");
write("  if(data->__activeOptions.size() > 0)\n");
write("    for(StateMachineBehaviorData::__Option** option = &data->__activeOptions[0], ** end = option + data->__activeOptions.size(); option < end; ++option)\n");
write("    {\n");
write("      const StateMachineBehaviorData::__Option& o = **option;\n");
write("      info.option = o.__id;\n");
write("      info.depth = o.__lastDepth;\n");
write("      info.state = o.__state;\n");
write("      info.optionTime = o.optionTime;\n");
write("      info.stateTime = o.stateTime;\n");
write("      options.push_back(info);\n");
write("    }\n");
write("}\n");
write("\n");


// decision and action code form each option
for(std::list<StateMachine::File>::const_iterator j = stateMachine.files.begin(), end = stateMachine.files.end(); j != end; ++j)
  for(std::list<StateMachine::Option>::const_iterator i = j->options.begin(), end = j->options.end(); i != end; ++i)
  {
    const StateMachine::Option& option = *i;
    write("unsigned int StateMachineBehaviorData::__" + option.name + "Option::__decision()\n");
    write("{\n");
    unsigned int line = 0;
    for(std::list<StateMachine::CppCode>::const_iterator i = option.commonDecisionCode.begin(), end = option.commonDecisionCode.end(); i != end; ++i)
    {
      const StateMachine::CppCode& cppCode = *i;
      if(useLineDirective)
      {
        if(line == 0)
          write(std::string("#line ") + itoa(cppCode.line) + " \"" + cppCode.file + "\"\n");
        else if(cppCode.line != line + 1)
          write(std::string("#line ") + itoa(cppCode.line) + "\n");
      }
      line = cppCode.line;
      write(cppCode.code + "\n");
    }
    if(option.states.size() > 0)
    {
      write("  switch(__state)\n");
      write("  {\n");
      for(std::list<StateMachine::Option::State>::const_iterator i = option.states.begin(), end = option.states.end(); i != end; ++i)
      {
        const StateMachine::Option::State& state = *i;
        write("  case " + state.name + ":\n");
        write("    {\n");
        unsigned int line = 0;
        for(std::list<StateMachine::CppCode>::const_iterator i = state.decisionCode.begin(), end = state.decisionCode.end(); i != end; ++i)
        {
          const StateMachine::CppCode& cppCode = *i;
          if(useLineDirective)
          {
            if(line == 0)
              write(std::string("#line ") + itoa(cppCode.line) + " \"" + cppCode.file + "\"\n");
            else if(cppCode.line != line + 1)
              write(std::string("#line ") + itoa(cppCode.line) + "\n");
          }
          line = cppCode.line;
          write(cppCode.code + "\n");
        }
        write("    }\n");
        write("    break;\n");
      }
      write("  }\n");
    }
    write("  return __state;\n");
    write("}\n");
    write("\n");

    write("unsigned int StateMachineBehaviorData::__" + option.name + "Option::__action()\n");
    write("{\n");
    line = 0;
    for(std::list<StateMachine::CppCode>::const_iterator i = option.commonActionCode.begin(), end = option.commonActionCode.end(); i != end; ++i)
    {
      const StateMachine::CppCode& cppCode = *i;
      if(useLineDirective)
      {
        if(line == 0)
          write(std::string("#line ") + itoa(cppCode.line) + " \"" + cppCode.file + "\"\n");
        else if(cppCode.line != line + 1)
          write(std::string("#line ") + itoa(cppCode.line) + "\n");
      }
      line = cppCode.line;
      write(cppCode.code + "\n");
    }
    if(option.states.size() > 0)
    {
      write("  switch(__state)\n");
      write("  {\n");
      for(std::list<StateMachine::Option::State>::const_iterator i = option.states.begin(), end = option.states.end(); i != end; ++i)
      {
        const StateMachine::Option::State& state = *i;
        write("  case " + state.name + ":\n");
        write("    {\n");
        unsigned int line = 0;
        for(std::list<StateMachine::CppCode>::const_iterator i = state.actionCode.begin(), end = state.actionCode.end(); i != end; ++i)
        {
          const StateMachine::CppCode& cppCode = *i;
          if(useLineDirective)
          {
            if(line == 0)
              write(std::string("#line ") + itoa(cppCode.line) + " \"" + cppCode.file + "\"\n");
            else if(cppCode.line != line + 1)
              write(std::string("#line ") + itoa(cppCode.line) + "\n");
          }
          line = cppCode.line;
          write(cppCode.code + "\n");
        }
        write("    }\n");
        write("    break;\n");
      }
      write("  }\n");
    }
    write("  return __state;\n");
    write("}\n");
    write("\n");
  }

    return closeFile();
  }

  bool generatePrototype(const StateMachine& stateMachine, const std::string& file)
  {
    if(!openFile(file))
      return false;

write("\n");
write("#pragma once\n");
write("\n");
write("#ifndef __DISABLE_STATE_MACHINE_PROTOTYPES\n");
write("\n");
write("#include \"StateMachineBehaviorEngine.h\"\n");
write("\n");
write("// input representations\n");
for(std::list<StateMachine::Input>::const_iterator i = stateMachine.input.begin(), end = stateMachine.input.end(); i != end; ++i)
  write(std::string("const ") + i->type + " " + i->name + ";\n");
write("\n");
write("// output representations\n");
for(std::list<StateMachine::Output>::const_iterator i = stateMachine.output.begin(), end = stateMachine.output.end(); i != end; ++i)
  write(i->type + " " + i->name + ";\n");
write("\n");
write("// special attributes\n");
write("unsigned int time;\n");
write("unsigned int optionTime;\n");
write("unsigned int stateTime;\n");
write("\n");
write("#define input class __Input\n");
write("#define output class __Output\n");
write("#define option class\n");
write("#define decision\n");
write("#define action\n");
write("class state\n");
write("{\n");
write("public:\n");
write("  template<typename A>state(A a) {}\n");
write("  state() {}\n");
write("  operator unsigned int() const {return 0;}\n");
write("};\n");
write("#define common state __common\n");
write("\n");
for(std::list<StateMachine::File>::const_iterator j = stateMachine.files.begin(), end = stateMachine.files.end(); j != end; ++j)
  for(std::list<StateMachine::Option>::const_iterator i = j->options.begin(), end = j->options.end(); i != end; ++i)
  {
    const StateMachine::Option& option = *i;
    write("class __" + option.name + "Option\n");
    write("{\n");
    write("public:\n");
    write("  enum __State\n");
    write("  {\n");
    for(std::list<StateMachine::Option::State>::const_iterator i = option.states.begin(), end = option.states.end(); i != end; ++i)
      write(i->name + ",\n");
    write("  };\n");
    write("\n");
    write("  unsigned int operator() () {}\n");
    write("\n");
    write("private:\n");
    for(std::list<StateMachine::CppCode>::const_iterator i = option.code.begin(), end = option.code.end(); i != end; ++i)
    {
      const StateMachine::CppCode& cppCode = *i;
      write(cppCode.code + "\n");
    }
    write("} " + option.name + ";\n");
    write("\n");
  }
write("#endif\n");

    return closeFile();
  }

private:
  bool useLineDirective;
  FILE* fp; /**< The currently opened file */
  std::string currentFile; /**< The name of the file currently opened */
  char buf[65]; /**< A buffer used to convert numbers into strings */

  bool openFile(const std::string& file)
  {
    if(fp)
      fclose(fp);
    fp = fopen(file.c_str(), "wb");
    if(!fp)
    {
      Error::print(file, 0, "error: could not open file");
      return false;
    }
    currentFile = file;
    return true;
  }

  bool closeFile()
  {
    if(!fp)
      return false;
    fclose(fp);
    fp = 0;
    return true;
  }

  bool write(const std::string& data)
  {
    if(!fp)
      return false;
    if(fwrite(data.c_str(), 1, data.size(), fp) != data.size())
    {
      fp = 0;
      Error::print(currentFile, 0, "error: could not write to file");
      return false;
    }
    return true;
  }
  
  const char* itoa(unsigned int i)
  {
    sprintf(buf, "%u", i);
    return buf;
  }
};
