
class Parser
{
public:
  Parser() : fp(0), readBufferPos(0), readBufferEnd(0) {}

  ~Parser()
  {
    if(fp)
      fclose(fp);
  }

  bool parse(const std::string& file, StateMachine& stateMachine)
  {
    if(fp)
      fclose(fp);
    fp = fopen(file.c_str(), "rb");
    if(!fp)
    {
      Error::print(file, 0, "error: could not open file");
      return false;
    }

    currentStateMachine = &stateMachine;
    stateMachine.files.push_back(StateMachine::File());
    currentFile = &stateMachine.files.back();
    currentFile->name = file;
    currentOption = 0;

    currentChar = 0;
    currentLine = 1;
    nextChar(); // read first char
    nextToken(); // read first token
    return readFile();
  }

private:
  class Token
  {
  public:
    enum Type
    {
      eof,
      leftBrace, rightBrace,
      leftParenthesis, rightParenthesis,
      semicolon,
      identifierOrKeyword,
      stringconst, charconst,
      preprocessorCode, unknownCppCode,
      numOfTypes,
    };

    static const char* getName(Type type)
    {
      static const char* typeNames[] = {
        "end of file",
        "'{'", "'}'",
        "'('", "')'",
        "';'",
        "identifier",
        "string", "char",
        "", "",
      };
      return typeNames[type];
    }

    enum Keyword
    {
      input, output,
      option, state, common,
      decision, action,
      numOfKeywords,
      unknown,
    };

    static const char* getKeywordName(Keyword keyword)
    {
      static const char* keywordNames[] = {
        "input", "output",
        "option", "state", "common",
        "decision", "action",
      };
      return keywordNames[keyword];
    }

    Type type;
    unsigned int line;
    std::string value;
    std::string passedSpace; /*< passed space characters since last token (without comments) */

    Keyword getKeyword() const
    {
      if(type != identifierOrKeyword)
        return unknown;

      static const std::tr1::unordered_map<std::string, Keyword>* keywords = 0;
      if(!keywords)
      {
        static std::tr1::unordered_map<std::string, Keyword> keywordHash;
        for(int i = 0; i < numOfKeywords; ++i)
          keywordHash[getKeywordName(Keyword(i))] = Keyword(i);
        keywords = &keywordHash;
      }
      std::tr1::unordered_map<std::string, Keyword>::const_iterator i = keywords->find(value);
      if(i == keywords->end())
        return unknown;
      return i->second;
    }
  };

  StateMachine* currentStateMachine;
  StateMachine::File* currentFile;
  StateMachine::Option* currentOption;
  FILE* fp;
  char readBuffer[1024 * 64];
  char* readBufferPos;
  char* readBufferEnd;

  char currentChar;
  unsigned int currentLine;
  Token currentToken;

  void nextChar()
  {
    if(readBufferPos >= readBufferEnd)
    {
      if(!fp)
      {
        currentChar = 0;
        return;
      }
      int i = fread(readBuffer, 1, sizeof(readBuffer), fp);
      if(i <= 0)
      {
        currentChar = 0;
        if(ferror(fp) != 0)
        {
          Error::print(currentFile->name, 0, "error: could not read from file");
          fclose(fp);
          fp = 0;
        }
        return;
      }
      readBufferPos = readBuffer;
      readBufferEnd = readBuffer + i;
    }
    currentChar = *(readBufferPos++);
  }

  void nextLine()
  {
    assert(currentChar == '\r' || currentChar == '\n');
    ++currentLine;
    if(currentChar == '\r')
    {
      nextChar();
      if(currentChar == '\n')
        nextChar();
    }
    else
      nextChar();
  }

  void nextToken()
  {
    currentToken.passedSpace.clear();
    currentToken.value.clear();
    currentToken.line = currentLine;
    for(;;)
// restart:
      switch(currentChar)
      {
      case 0: currentToken.type = Token::eof; return;
      case '{': nextChar(); currentToken.value = "{"; currentToken.type = Token::leftBrace; return;
      case '}': nextChar(); currentToken.value = "}"; currentToken.type = Token::rightBrace; return;
      case '(': nextChar(); currentToken.value = "("; currentToken.type = Token::leftParenthesis; return;
      case ')': nextChar(); currentToken.value = ")"; currentToken.type = Token::rightParenthesis; return;
      case ';': nextChar(); currentToken.value = ";"; currentToken.type = Token::semicolon; return;

      case '/':
        nextChar();
        switch(currentChar)
        {
        case '*': // "/*"
          nextChar();
          for(;;) // find comment terminator
          {
            switch(currentChar)
            {
            case 0: break;
            case '*':
              nextChar();
              if(currentChar == '/')
              {
                nextChar();
                break;
              }
              continue;
            case '\r':
            case '\n':
              nextLine();
              currentToken.passedSpace.clear();
              currentToken.line = currentLine;
              continue;
            default:
              nextChar();
              continue;
            }
            break;
          }
          continue; // goto restart

        case '/': // "//"
          nextChar();
          for(;;) // find line ending
          {
            switch(currentChar)
            {
            case 0: break;
            case '\\':
              nextChar();
              if(currentChar == '\r' || currentChar == '\n')
              {
                nextLine();
                currentToken.passedSpace.clear();
                currentToken.line = currentLine;
              }
              continue;
            case '\r':
            case '\n':
              nextLine();
              currentToken.passedSpace.clear();
              currentToken.line = currentLine;
              break;
            default:
              nextChar();
              continue;
            }
            break;
          }
          continue; // goto restart

        default:
          currentToken.value = "/"; currentToken.type = Token::unknownCppCode; return;
        }
        assert(false);
        break;

      case '#':
        currentToken.value += '#';
        currentToken.type = Token::preprocessorCode;
        nextChar();
        for(;;) // find line ending
        {
          switch(currentChar)
          {
          case 0: break;
          case '\\':
            currentToken.value += currentChar;
            nextChar();
            if(currentChar == '\r' || currentChar == '\n')
            {
              currentToken.value += '\n';
              nextLine();
            }
            continue;
          case '\r':
          case '\n':
            nextLine();
            break;
          default:
            currentToken.value += currentChar;
            nextChar();
            continue;
          }
          break;
        }
        return;

      case '"':
      case '\'':
        {
          currentToken.value += currentChar;
          currentToken.type = currentChar == '"' ? Token::stringconst : Token::charconst;
          char quoteChar = currentChar;
          nextChar();
          for(;;) // find string ending
          {
            switch(currentChar)
            {
            case 0: break;
            case '\\':
              currentToken.value += currentChar;
              nextChar();
              if(currentChar == quoteChar)
              {
                currentToken.value += currentChar;
                nextChar();
              }
              continue;
            case '\r':
            case '\n':
              currentToken.value += '\n';
              nextLine();
              continue;
            default:
              currentToken.value += currentChar;
              nextChar();
              if(currentChar == quoteChar)
              {
                currentToken.value += currentChar;
                nextChar();
                break;
              }
              else
                continue;
            }
            break;
          }
        }
        return;

      case ' ':
      case '\t':
      case '\v':
      case '\f':
        currentToken.passedSpace += currentChar;
        nextChar();
        continue;

      case '\r':
      case '\n':
        nextLine();
        currentToken.passedSpace.clear();
        currentToken.line = currentLine;
        continue;

      default:

        if(isalpha(currentChar) || currentChar == '_')
        {
          currentToken.type = Token::identifierOrKeyword;
          currentToken.value += currentChar;
          nextChar();
          for(;;)
            if(isalnum(currentChar) || currentChar == '_')
            {
              currentToken.value += currentChar;
              nextChar();
              continue;
            }
            else
              return;
          assert(false);
          break;
        }

        currentToken.type = Token::unknownCppCode;
        currentToken.value += currentChar;
        nextChar();
        return;
      }
  }

  void errorExcepted(const char* what)
  {
    if(!fp)
      return;
    std::string got;
    Token::Keyword keyword = currentToken.getKeyword();
    if(keyword != Token::unknown)
      got = Token::getKeywordName(keyword);
    else
    {
      got = Token::getName(currentToken.type);
      if(got.empty())
        got = *currentToken.value.c_str();
    }
    Error::print(currentFile->name, currentToken.line, "error: unexpected %s (expected %s)", got.c_str(), what);
  }

  void errorUnexpectedToken()
  {
    if(!fp)
      return;
    std::string got;
    Token::Keyword keyword = currentToken.getKeyword();
    if(keyword != Token::unknown)
      got = Token::getKeywordName(keyword);
    else
    {
      got = Token::getName(currentToken.type);
      if(got.empty())
        got = *currentToken.value.c_str();
    }
    Error::print(currentFile->name, currentToken.line, "error: unexpected %s", got.c_str());
  }

  bool expectToken(Token::Type type)
  {
    if(currentToken.type != type)
    {
      errorExcepted(Token::getName(type));
      return false;
    }
    nextToken();
    return true;
  }

  bool expectIdentifier(std::string& identifier)
  {
    if(currentToken.type != Token::identifierOrKeyword)
      return expectToken(Token::identifierOrKeyword);
    identifier = currentToken.value;
    nextToken();
    return true;
  }

  // file = { line }
  bool readFile()
  {
    while(currentToken.type != Token::eof)
      if(!readLine())
        return false;
    return true;
  }

  // line = 'input' input | 'ouput' output | 'option' option | cppCode
  bool readLine()
  {
    switch(currentToken.getKeyword())
    {
    case Token::input:
      nextToken();
      return readInput();
    case Token::output:
      nextToken();
      return readOutput();
    case Token::option:
      nextToken();
      return readOption();
    default:
      return readCppCode(currentFile->code);
    }
  }

  // input = '{' { identifier identifier ';' { ';' } } '}' ';'
  bool readInput()
  {
    if(!expectToken(Token::leftBrace))
      return false;

    while(currentToken.type != Token::rightBrace)
    {
      currentStateMachine->input.push_back(StateMachine::Input());
      StateMachine::Input& input = currentStateMachine->input.back();

      if(!expectIdentifier(input.type))
        return false;
      input.file = currentFile->name;
      input.line = currentToken.line;
      if(!expectIdentifier(input.name))
        return false;

      if(currentStateMachine->inputByName.find(input.name) != currentStateMachine->inputByName.end())
      {
        StateMachine::Input& alreadyDefinedInput = *currentStateMachine->inputByName[input.name];
        Error::print(currentFile->name, currentToken.line, "error: input %s already declared", input.name.c_str());
        Error::print(alreadyDefinedInput.file, alreadyDefinedInput.line, "see declaration of %s", alreadyDefinedInput.name.c_str());
        return false;
      }
      currentStateMachine->inputByName[input.name] = &input;

      if(!expectToken(Token::semicolon))
        return false;
      while(currentToken.type == Token::semicolon)
        nextToken();
    }
    nextToken();
    if(!expectToken(Token::semicolon))
      return false;
    return true;
  }

  // output = '{' { identifier identifier ';' { ';' } } '}' ';'
  bool readOutput()
  {
    if(!expectToken(Token::leftBrace))
      return false;

    while(currentToken.type != Token::rightBrace)
    {
      currentStateMachine->output.push_back(StateMachine::Output());
      StateMachine::Output& output = currentStateMachine->output.back();

      if(!expectIdentifier(output.type))
        return false;
      output.file = currentFile->name;
      output.line = currentToken.line;
      if(!expectIdentifier(output.name))
        return false;

      if(currentStateMachine->outputByName.find(output.name) != currentStateMachine->outputByName.end())
      {
        StateMachine::Output& alreadyDefinedOutput = *currentStateMachine->outputByName[output.name];
        Error::print(currentFile->name, currentToken.line, "error: output %s already declared", output.name.c_str());
        Error::print(alreadyDefinedOutput.file, alreadyDefinedOutput.line, "see declaration of %s", alreadyDefinedOutput.name.c_str());
        return false;
      }
      currentStateMachine->outputByName[output.name] = &output;

      if(!expectToken(Token::semicolon))
        return false;
      while(currentToken.type == Token::semicolon)
        nextToken();
    }
    nextToken();
    if(!expectToken(Token::semicolon))
      return false;
    return true;
  }

  // option = identifier '{' { optionLine } '}' ';'
  bool readOption()
  {
    currentFile->options.push_back(StateMachine::Option());
    currentOption = &currentFile->options.back();

    currentOption->file = currentFile->name;
    currentOption->line = currentToken.line;
    if(!expectIdentifier(currentOption->name))
      return false;

    if(currentFile->optionsByName.find(currentOption->name) != currentFile->optionsByName.end())
    {
      StateMachine::Option& alreadyDefinedOption = *currentFile->optionsByName[currentOption->name];
      Error::print(currentFile->name, currentToken.line, "error: option %s already declared", currentOption->name.c_str());
      Error::print(alreadyDefinedOption.file, alreadyDefinedOption.line, "see declaration of %s",  alreadyDefinedOption.name.c_str());
      return false;
    }
    currentFile->optionsByName[currentOption->name] = currentOption;

    if(!expectToken(Token::leftBrace))
      return false;

    while(currentToken.type != Token::rightBrace)
      if(!readOptionLine())
        return false;
    nextToken();

    if(!expectToken(Token::semicolon))
      return false;

    currentOption = 0;
    return true;
  }

  // optionLine = 'common' common | 'state' state | cppCode
  bool readOptionLine()
  {
    assert(currentOption);
    switch(currentToken.getKeyword())
    {
    case Token::state:
      nextToken();
      return readState();
    case Token::common:
      nextToken();
      return readCommon();
    default:
      if(currentToken.type == Token::identifierOrKeyword && currentToken.value == currentOption->name)
      {// constructor
        currentToken.value = "__" + currentOption->name + "Option";
        return readCppCode(currentOption->code);
      }
      else
        return readCppCode(currentOption->code);
    }
  }

  // common = '(' ')' '{' { 'decision' cppBody | 'action' cppBody } '}'
  bool readCommon()
  {
    assert(currentOption);

    if(!expectToken(Token::leftParenthesis))
      return false;
    if(!expectToken(Token::rightParenthesis))
      return false;
    if(!expectToken(Token::leftBrace))
      return false;
    while(currentToken.type != Token::rightBrace)
      switch(currentToken.getKeyword())
      {
      case Token::decision:
        nextToken();
        if(!readCppBody(currentOption->commonDecisionCode))
          return false;
        break;
      case Token::action:
        nextToken();
        if(!readCppBody(currentOption->commonActionCode))
          return false;
        break;
      default:
        errorExcepted("decision or action");
        return false;
      }
    nextToken();
    return true;
  }

  // state = identifier ( ';' | '(' ')' '{' { 'decision' cppBody | 'action' cppBody } '}' )
  bool readState()
  {
    assert(currentOption);

    std::string name;
    unsigned int line = currentToken.line;
    if(!expectIdentifier(name))
      return false;

    switch(currentToken.type)
    {
    case Token::semicolon:
      nextToken();
      addCppCode(line, "state " + name + ";", currentOption->code);
      return true;
    
    case Token::leftParenthesis:

      {
        currentOption->states.push_back(StateMachine::Option::State());
        StateMachine::Option::State& state = currentOption->states.back();

        state.name = name;
        state.file = currentFile->name;
        state.line = line;

        if(currentOption->statesByName.find(state.name) != currentOption->statesByName.end())
        {
          StateMachine::Option::State& alreadyDefinedState = *currentOption->statesByName[state.name];
          Error::print(currentFile->name, currentToken.line, "error: state %s already declared", state.name.c_str());
          Error::print(alreadyDefinedState.file, alreadyDefinedState.line, "see declaration of %s", alreadyDefinedState.name.c_str());
          return false;
        }
        currentOption->statesByName[state.name] = &state;

        nextToken();
        if(!expectToken(Token::rightParenthesis))
          return false;

        if(!expectToken(Token::leftBrace))
          return false;

        while(currentToken.type != Token::rightBrace)
          switch(currentToken.getKeyword())
          {
          case Token::decision:
            nextToken();
            if(!readCppBody(state.decisionCode))
              return false;
            break;
          case Token::action:
            nextToken();
            if(!readCppBody(state.actionCode))
              return false;
            break;
          default:
            errorExcepted("decision or action");
            return false;
          }
        nextToken();
        return true;
      }

    default:
      errorExcepted("'(' or ';'");
      return false;
    }
  }

  // cppBody = '{' { cppCode } '}'
  bool readCppBody(std::list<StateMachine::CppCode>& cppCode)
  {
    if(!expectToken(Token::leftBrace))
      return false;
    while(currentToken.type != Token::rightBrace)
      if(!readCppCode(cppCode))
        return false;
    nextToken();
    return true;
  }

  // cppCode = '{' { cppCode } '}' | '(' { cppCode } ')' | "everthing except '{', '(' and eof"
  bool readCppCode(std::list<StateMachine::CppCode>& cppCode)
  {
    switch(currentToken.type)
    {
    case Token::leftBrace:
      addTokenToCppCode(cppCode);
      nextToken();
      while(currentToken.type != Token::rightBrace)
        if(!readCppCode(cppCode))
          return false;
      addTokenToCppCode(cppCode);
      nextToken();
      return true;;
    case Token::leftParenthesis:
      addTokenToCppCode(cppCode);
      nextToken();
      while(currentToken.type != Token::rightParenthesis)
        if(!readCppCode(cppCode))
          return false;
      addTokenToCppCode(cppCode);
      nextToken();
      return true;;

    case Token::eof:
      errorUnexpectedToken();
      return false;

    default:
      addTokenToCppCode(cppCode);
      nextToken();
      return true;;
    }
  }

  void addCppCode(unsigned int line, const std::string& additionalCode, std::list<StateMachine::CppCode>& cppCode) const
  {
    StateMachine::CppCode* lastCppCode = !cppCode.empty() ? &cppCode.back() : 0;
    if(!lastCppCode || lastCppCode->line != line)
    {
      cppCode.push_back(StateMachine::CppCode());
      lastCppCode = &cppCode.back();
      lastCppCode->file = currentFile->name;
      lastCppCode->line = line;
    }
    lastCppCode->code += additionalCode;
  }

  void addTokenToCppCode(std::list<StateMachine::CppCode>& cppCode) const
  {
    addCppCode(currentToken.line, currentToken.passedSpace + currentToken.value, cppCode);
  }

};

