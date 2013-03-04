
class Error
{
public:
  static void print(const std::string& file, int line, const char* format, ...)
  {
#ifdef WIN32
    if(line)
      fprintf(stderr, "%s(%u): ", file.c_str(), line);
    else
      fprintf(stderr, "%s: ", file.c_str());
#else
    if(line)
      fprintf(stderr, "%s:%u: ", file.c_str(), line);
    else
      fprintf(stderr, "%s: ", file.c_str());
#endif
    va_list ap;
    va_start(ap, format); 
    vfprintf(stderr, format, ap);
    va_end(ap);
    fputc('\n', stderr);
  }
};
