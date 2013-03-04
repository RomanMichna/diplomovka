

# This file was generated by zbuildgen. Don't edit this file! Edit "SimRobotEditor.make.zbuild" instead.


SOLUTIONDIR ?= $(CURDIR)

ifeq ($(CONFIG),Debug)

PROJECTNAME := SimRobotEditor
CONFIG := Debug
OUTDIR := $(SOLUTIONDIR)/../Build/SimRobotEditor/Linux/$(CONFIG)
INTDIR := $(SOLUTIONDIR)/../Build/SimRobotEditor/Linux/$(CONFIG)
TARGET := $(OUTDIR)/libSimRobotEditor.so
OBJECTS := \
	$(INTDIR)/moc_EditorWidget.o\
	$(INTDIR)/moc_SyntaxHighlighter.o\
	$(INTDIR)/qrc_SimRobotEditor.o\
	$(INTDIR)/EditorModule.o\
	$(INTDIR)/EditorWidget.o\
	$(INTDIR)/SyntaxHighlighter.o\

PCHOBJECT := 
CUSTOMOBJECTS := \
	$(INTDIR)/moc_EditorWidget.cpp\
	$(INTDIR)/qrc_SimRobotEditor.cpp\
	$(INTDIR)/moc_SyntaxHighlighter.cpp\

DEPENDENCIES := 

DEFINES := -D"LINUX" -D"QT_NO_DEBUG" -D"QT_SHARED" -D"QT_GUI_LIB" -D"QT_CORE_LIB" -D"DEBUG" -D"_DEBUG"
INCPATHS := -I"$(INTDIR)" -I"../Src/SimRobotEditor" -I"/usr/include/qt4/QtCore" -I"/usr/include/qt4/QtGui" -I"/usr/include/qt4"
LIBPATHS := 
LIBS := -lrt -lpthread -lQtGui -lQtCore

BUILDFLAGS := -MMD $(DEFINES) $(INCPATHS)   -fpic -pipe -Wall -W -Wno-unused-parameter -Wno-non-virtual-dtor -Wno-deprecated -Wno-ignored-qualifiers -g
LINKFLAGS :=  -shared -fpic  $(LIBPATHS) $(LIBS)

.PHONY: all prebuild clean dist distclean

all: $(TARGET)

$(TARGET): $(OBJECTS) $(CUSTOMOBJECTS) $(DEPENDENCIES)
	@echo Linking...
	@$(CXX) -o $@ $(OBJECTS) $(LDFLAGS) $(LINKFLAGS) 

$(INTDIR)/moc_EditorWidget.o: $(INTDIR)/moc_EditorWidget.cpp  | prebuild
	@echo "moc_EditorWidget.cpp"
	@$(CXX) $(CXXFLAGS) $(BUILDFLAGS) -o $@ -c $<

$(INTDIR)/moc_SyntaxHighlighter.o: $(INTDIR)/moc_SyntaxHighlighter.cpp  | prebuild
	@echo "moc_SyntaxHighlighter.cpp"
	@$(CXX) $(CXXFLAGS) $(BUILDFLAGS) -o $@ -c $<

$(INTDIR)/qrc_SimRobotEditor.o: $(INTDIR)/qrc_SimRobotEditor.cpp  | prebuild
	@echo "qrc_SimRobotEditor.cpp"
	@$(CXX) $(CXXFLAGS) $(BUILDFLAGS) -o $@ -c $<

$(INTDIR)/EditorModule.o: ../Src/SimRobotEditor/EditorModule.cpp  | prebuild
	@echo "EditorModule.cpp"
	@$(CXX) $(CXXFLAGS) $(BUILDFLAGS) -o $@ -c $<

$(INTDIR)/EditorWidget.o: ../Src/SimRobotEditor/EditorWidget.cpp  | prebuild
	@echo "EditorWidget.cpp"
	@$(CXX) $(CXXFLAGS) $(BUILDFLAGS) -o $@ -c $<

$(INTDIR)/SyntaxHighlighter.o: ../Src/SimRobotEditor/SyntaxHighlighter.cpp  | prebuild
	@echo "SyntaxHighlighter.cpp"
	@$(CXX) $(CXXFLAGS) $(BUILDFLAGS) -o $@ -c $<

$(INTDIR)/moc_EditorWidget.cpp: ../Src/SimRobotEditor/EditorWidget.h  | prebuild
	@echo "EditorWidget.h (Qt moc)"
	@moc-qt4 -DLINUX -DQT_NO_DEBUG -DQT_SHARED -DQT_GUI_LIB -DQT_CORE_LIB -DDEBUG -D_DEBUG -I../Src/SimRobotEditor -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4 ../Src/SimRobotEditor/EditorWidget.h -o $(INTDIR)/moc_EditorWidget.cpp

$(INTDIR)/qrc_SimRobotEditor.cpp: ../Src/SimRobotEditor/SimRobotEditor.qrc ../Src/SimRobotEditor/Icons/page_white_stack.png ../Src/SimRobotEditor/Icons/page_white_text.png ../Src/SimRobotEditor/SimRobotEditor.qrc | prebuild
	@echo "SimRobotEditor.qrc (Qt rcc)"
	@rcc -name SimRobotEditor ../Src/SimRobotEditor/SimRobotEditor.qrc -o $(INTDIR)/qrc_SimRobotEditor.cpp

$(INTDIR)/moc_SyntaxHighlighter.cpp: ../Src/SimRobotEditor/SyntaxHighlighter.h  | prebuild
	@echo "SyntaxHighlighter.h (Qt moc)"
	@moc-qt4 -DLINUX -DQT_NO_DEBUG -DQT_SHARED -DQT_GUI_LIB -DQT_CORE_LIB -DDEBUG -D_DEBUG -I../Src/SimRobotEditor -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4 ../Src/SimRobotEditor/SyntaxHighlighter.h -o $(INTDIR)/moc_SyntaxHighlighter.cpp

$(INTDIR):
	@mkdir -p $(INTDIR)

ifneq ($(INTDIR),$(OUTDIR))
$(OUTDIR):
	@mkdir -p $(OUTDIR)
endif

prebuild: $(INTDIR) $(OUTDIR)
	@echo "------ Building $(PROJECTNAME) ($(CONFIG)) ------"

clean:
	@echo "------ Cleaning $(PROJECTNAME) ($(CONFIG)) ------"
	@echo Cleaning...
	-@rm -rf $(OBJECTS) $(PCHOBJECT) $(CUSTOMOBJECTS) $(TARGET) $(OBJECTS:%.o=%.d) $(PCHOBJECT:%.gch=%.d)

-include $(OBJECTS:%.o=%.d)

else

PROJECTNAME := SimRobotEditor
CONFIG := Release
OUTDIR := $(SOLUTIONDIR)/../Build/SimRobotEditor/Linux/$(CONFIG)
INTDIR := $(SOLUTIONDIR)/../Build/SimRobotEditor/Linux/$(CONFIG)
TARGET := $(OUTDIR)/libSimRobotEditor.so
OBJECTS := \
	$(INTDIR)/moc_EditorWidget.o\
	$(INTDIR)/moc_SyntaxHighlighter.o\
	$(INTDIR)/qrc_SimRobotEditor.o\
	$(INTDIR)/EditorModule.o\
	$(INTDIR)/EditorWidget.o\
	$(INTDIR)/SyntaxHighlighter.o\

PCHOBJECT := 
CUSTOMOBJECTS := \
	$(INTDIR)/moc_EditorWidget.cpp\
	$(INTDIR)/qrc_SimRobotEditor.cpp\
	$(INTDIR)/moc_SyntaxHighlighter.cpp\

DEPENDENCIES := 

DEFINES := -D"LINUX" -D"QT_NO_DEBUG" -D"QT_SHARED" -D"QT_GUI_LIB" -D"QT_CORE_LIB" -D"NDEBUG"
INCPATHS := -I"$(INTDIR)" -I"../Src/SimRobotEditor" -I"/usr/include/qt4/QtCore" -I"/usr/include/qt4/QtGui" -I"/usr/include/qt4"
LIBPATHS := 
LIBS := -lrt -lpthread -lQtGui -lQtCore

BUILDFLAGS := -MMD $(DEFINES) $(INCPATHS)   -fpic -pipe -Wall -W -Wno-unused-parameter -Wno-non-virtual-dtor -Wno-deprecated -Wno-ignored-qualifiers -Wno-unused -O3 -fomit-frame-pointer
LINKFLAGS :=  -shared -fpic -s $(LIBPATHS) $(LIBS)

.PHONY: all prebuild clean dist distclean

all: $(TARGET)

$(TARGET): $(OBJECTS) $(CUSTOMOBJECTS) $(DEPENDENCIES)
	@echo Linking...
	@$(CXX) -o $@ $(OBJECTS) $(LDFLAGS) $(LINKFLAGS) 

$(INTDIR)/moc_EditorWidget.o: $(INTDIR)/moc_EditorWidget.cpp  | prebuild
	@echo "moc_EditorWidget.cpp"
	@$(CXX) $(CXXFLAGS) $(BUILDFLAGS) -o $@ -c $<

$(INTDIR)/moc_SyntaxHighlighter.o: $(INTDIR)/moc_SyntaxHighlighter.cpp  | prebuild
	@echo "moc_SyntaxHighlighter.cpp"
	@$(CXX) $(CXXFLAGS) $(BUILDFLAGS) -o $@ -c $<

$(INTDIR)/qrc_SimRobotEditor.o: $(INTDIR)/qrc_SimRobotEditor.cpp  | prebuild
	@echo "qrc_SimRobotEditor.cpp"
	@$(CXX) $(CXXFLAGS) $(BUILDFLAGS) -o $@ -c $<

$(INTDIR)/EditorModule.o: ../Src/SimRobotEditor/EditorModule.cpp  | prebuild
	@echo "EditorModule.cpp"
	@$(CXX) $(CXXFLAGS) $(BUILDFLAGS) -o $@ -c $<

$(INTDIR)/EditorWidget.o: ../Src/SimRobotEditor/EditorWidget.cpp  | prebuild
	@echo "EditorWidget.cpp"
	@$(CXX) $(CXXFLAGS) $(BUILDFLAGS) -o $@ -c $<

$(INTDIR)/SyntaxHighlighter.o: ../Src/SimRobotEditor/SyntaxHighlighter.cpp  | prebuild
	@echo "SyntaxHighlighter.cpp"
	@$(CXX) $(CXXFLAGS) $(BUILDFLAGS) -o $@ -c $<

$(INTDIR)/moc_EditorWidget.cpp: ../Src/SimRobotEditor/EditorWidget.h  | prebuild
	@echo "EditorWidget.h (Qt moc)"
	@moc-qt4 -DLINUX -DQT_NO_DEBUG -DQT_SHARED -DQT_GUI_LIB -DQT_CORE_LIB -DNDEBUG -I../Src/SimRobotEditor -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4 ../Src/SimRobotEditor/EditorWidget.h -o $(INTDIR)/moc_EditorWidget.cpp

$(INTDIR)/qrc_SimRobotEditor.cpp: ../Src/SimRobotEditor/SimRobotEditor.qrc ../Src/SimRobotEditor/Icons/page_white_stack.png ../Src/SimRobotEditor/Icons/page_white_text.png ../Src/SimRobotEditor/SimRobotEditor.qrc | prebuild
	@echo "SimRobotEditor.qrc (Qt rcc)"
	@rcc -name SimRobotEditor ../Src/SimRobotEditor/SimRobotEditor.qrc -o $(INTDIR)/qrc_SimRobotEditor.cpp

$(INTDIR)/moc_SyntaxHighlighter.cpp: ../Src/SimRobotEditor/SyntaxHighlighter.h  | prebuild
	@echo "SyntaxHighlighter.h (Qt moc)"
	@moc-qt4 -DLINUX -DQT_NO_DEBUG -DQT_SHARED -DQT_GUI_LIB -DQT_CORE_LIB -DNDEBUG -I../Src/SimRobotEditor -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4 ../Src/SimRobotEditor/SyntaxHighlighter.h -o $(INTDIR)/moc_SyntaxHighlighter.cpp

$(INTDIR):
	@mkdir -p $(INTDIR)

ifneq ($(INTDIR),$(OUTDIR))
$(OUTDIR):
	@mkdir -p $(OUTDIR)
endif

prebuild: $(INTDIR) $(OUTDIR)
	@echo "------ Building $(PROJECTNAME) ($(CONFIG)) ------"

clean:
	@echo "------ Cleaning $(PROJECTNAME) ($(CONFIG)) ------"
	@echo Cleaning...
	-@rm -rf $(OBJECTS) $(PCHOBJECT) $(CUSTOMOBJECTS) $(TARGET) $(OBJECTS:%.o=%.d) $(PCHOBJECT:%.gch=%.d)

-include $(OBJECTS:%.o=%.d)

endif

%.h: ;
%.hh: ;
%.hxx: ;
%.hpp: ;
%.c: ;
%.cc: ;
%.cxx: ;
%.cpp: ;
%.d: ;

dist: ;

distclean: ;
