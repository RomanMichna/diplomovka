

# This file was generated by zbuildgen. Don't edit this file! Edit "SimRobotHelp.make.zbuild" instead.


SOLUTIONDIR ?= $(CURDIR)

ifeq ($(CONFIG),Debug)

PROJECTNAME := SimRobotHelp
CONFIG := Debug
OUTDIR := $(SOLUTIONDIR)/../Build/SimRobotHelp/Linux/$(CONFIG)
INTDIR := $(SOLUTIONDIR)/../Build/SimRobotHelp/Linux/$(CONFIG)
TARGET := $(OUTDIR)/libSimRobotHelp.so
OBJECTS := \
	$(INTDIR)/moc_HelpWidget.o\
	$(INTDIR)/qrc_SimRobotHelp.o\
	$(INTDIR)/HelpModule.o\
	$(INTDIR)/HelpWidget.o\

PCHOBJECT := 
CUSTOMOBJECTS := \
	$(INTDIR)/qrc_SimRobotHelp.cpp\
	../Build/SimRobotHelp/Help/help.qch\
	../Build/SimRobotHelp/Help/helpcollection.qhc\
	$(INTDIR)/moc_HelpWidget.cpp\

DEPENDENCIES := 

DEFINES := -D"LINUX" -D"QT_NO_DEBUG" -D"QT_SHARED" -D"QT_GUI_LIB" -D"QT_CORE_LIB" -D"QT_HELP_LIB" -D"DEBUG" -D"_DEBUG"
INCPATHS := -I"$(INTDIR)" -I"../Src/SimRobotHelp" -I"/usr/include/qt4/QtCore" -I"/usr/include/qt4/QtGui" -I"/usr/include/qt4/QtHelp" -I"/usr/include/qt4"
LIBPATHS := 
LIBS := -lrt -lpthread -lQtGui -lQtCore -lQtHelp

BUILDFLAGS := -MMD $(DEFINES) $(INCPATHS)   -fpic -pipe -Wall -W -Wno-unused-parameter -Wno-non-virtual-dtor -Wno-deprecated -Wno-ignored-qualifiers -g
LINKFLAGS :=  -shared -fpic  $(LIBPATHS) $(LIBS)

.PHONY: all prebuild clean dist distclean

all: $(TARGET)

$(TARGET): $(OBJECTS) $(CUSTOMOBJECTS) $(DEPENDENCIES)
	@echo Linking...
	@$(CXX) -o $@ $(OBJECTS) $(LDFLAGS) $(LINKFLAGS) 

$(INTDIR)/moc_HelpWidget.o: $(INTDIR)/moc_HelpWidget.cpp  | prebuild
	@echo "moc_HelpWidget.cpp"
	@$(CXX) $(CXXFLAGS) $(BUILDFLAGS) -o $@ -c $<

$(INTDIR)/qrc_SimRobotHelp.o: $(INTDIR)/qrc_SimRobotHelp.cpp  | prebuild
	@echo "qrc_SimRobotHelp.cpp"
	@$(CXX) $(CXXFLAGS) $(BUILDFLAGS) -o $@ -c $<

$(INTDIR)/HelpModule.o: ../Src/SimRobotHelp/HelpModule.cpp  | prebuild
	@echo "HelpModule.cpp"
	@$(CXX) $(CXXFLAGS) $(BUILDFLAGS) -o $@ -c $<

$(INTDIR)/HelpWidget.o: ../Src/SimRobotHelp/HelpWidget.cpp  | prebuild
	@echo "HelpWidget.cpp"
	@$(CXX) $(CXXFLAGS) $(BUILDFLAGS) -o $@ -c $<

$(INTDIR)/qrc_SimRobotHelp.cpp: ../Src/SimRobotHelp/SimRobotHelp.qrc ../Src/SimRobotHelp/Help/Icons/SimRobot.png ../Src/SimRobotHelp/Help/Icons/app_exit.bmp ../Src/SimRobotHelp/Help/Icons/app_exit_xp.bmp ../Src/SimRobotHelp/Help/Icons/architecture.png ../Src/SimRobotHelp/Help/Icons/crystal_toolbar.bmp ../Src/SimRobotHelp/Help/Icons/status_bar.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_camera.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_clock.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_color_image.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_column_graph.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_copy.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_cut.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_drag_and_drop_plane.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_grid.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_help.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_light.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_line_graph.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_monochrome_image.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_motion_blur.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_new.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_open.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_paste.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_reset.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_save.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_search.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_shader_interface.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_show_sensors.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_start.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_step.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_stereogram.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_surface_rendering.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_tree_view.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_update_view.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_view_item.bmp ../Src/SimRobotHelp/Help/api/api.html ../Src/SimRobotHelp/Help/basics/architecture.html ../Src/SimRobotHelp/Help/basics/basics.html ../Src/SimRobotHelp/Help/basics/physics.html ../Src/SimRobotHelp/Help/basics/quickstart.html ../Src/SimRobotHelp/Help/basics/sensing.html ../Src/SimRobotHelp/Help/basics/specification.html ../Src/SimRobotHelp/Help/console/commands.html ../Src/SimRobotHelp/Help/examples/examples.html ../Src/SimRobotHelp/Help/faq/faq.html ../Src/SimRobotHelp/Help/help.qhp ../Src/SimRobotHelp/Help/helpcollection.qhcp ../Src/SimRobotHelp/Help/how_to_help.html ../Src/SimRobotHelp/Help/index.html ../Src/SimRobotHelp/Help/interface/console.html ../Src/SimRobotHelp/Help/interface/editmenu.html ../Src/SimRobotHelp/Help/interface/editorwindow.html ../Src/SimRobotHelp/Help/interface/filemenu.html ../Src/SimRobotHelp/Help/interface/helpmenu.html ../Src/SimRobotHelp/Help/interface/interface.html ../Src/SimRobotHelp/Help/interface/menus.html ../Src/SimRobotHelp/Help/interface/objectwindow.html ../Src/SimRobotHelp/Help/interface/sensorwindow.html ../Src/SimRobotHelp/Help/interface/simulationmenu.html ../Src/SimRobotHelp/Help/interface/statusbar.html ../Src/SimRobotHelp/Help/interface/toolbar.html ../Src/SimRobotHelp/Help/interface/treewindow.html ../Src/SimRobotHelp/Help/interface/viewmenu.html ../Src/SimRobotHelp/Help/interface/windowmenu.html ../Src/SimRobotHelp/Help/rosiml/rosi.html ../Src/SimRobotHelp/Help/rosiml/rosi_controller.html ../Src/SimRobotHelp/Help/simrobothelp.css ../Src/SimRobotHelp/Icons/back.png ../Src/SimRobotHelp/Icons/forward.png ../Src/SimRobotHelp/Icons/help.png ../Src/SimRobotHelp/Icons/home.png ../Src/SimRobotHelp/Icons/locate.png ../Src/SimRobotHelp/SimRobotHelp.qrc | prebuild
	@echo "SimRobotHelp.qrc (Qt rcc)"
	@rcc -name SimRobotHelp ../Src/SimRobotHelp/SimRobotHelp.qrc -o $(INTDIR)/qrc_SimRobotHelp.cpp

../Build/SimRobotHelp/Help/help.qch: ../Src/SimRobotHelp/Help/help.qhp ../Src/SimRobotHelp/Help/Icons/SimRobot.png ../Src/SimRobotHelp/Help/Icons/app_exit.bmp ../Src/SimRobotHelp/Help/Icons/app_exit_xp.bmp ../Src/SimRobotHelp/Help/Icons/architecture.png ../Src/SimRobotHelp/Help/Icons/crystal_toolbar.bmp ../Src/SimRobotHelp/Help/Icons/status_bar.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_camera.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_clock.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_color_image.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_column_graph.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_copy.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_cut.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_drag_and_drop_plane.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_grid.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_help.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_light.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_line_graph.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_monochrome_image.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_motion_blur.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_new.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_open.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_paste.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_reset.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_save.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_search.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_shader_interface.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_show_sensors.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_start.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_step.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_stereogram.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_surface_rendering.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_tree_view.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_update_view.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_view_item.bmp ../Src/SimRobotHelp/Help/api/api.html ../Src/SimRobotHelp/Help/basics/architecture.html ../Src/SimRobotHelp/Help/basics/basics.html ../Src/SimRobotHelp/Help/basics/physics.html ../Src/SimRobotHelp/Help/basics/quickstart.html ../Src/SimRobotHelp/Help/basics/sensing.html ../Src/SimRobotHelp/Help/basics/specification.html ../Src/SimRobotHelp/Help/console/commands.html ../Src/SimRobotHelp/Help/examples/examples.html ../Src/SimRobotHelp/Help/faq/faq.html ../Src/SimRobotHelp/Help/help.qhp ../Src/SimRobotHelp/Help/helpcollection.qhcp ../Src/SimRobotHelp/Help/how_to_help.html ../Src/SimRobotHelp/Help/index.html ../Src/SimRobotHelp/Help/interface/console.html ../Src/SimRobotHelp/Help/interface/editmenu.html ../Src/SimRobotHelp/Help/interface/editorwindow.html ../Src/SimRobotHelp/Help/interface/filemenu.html ../Src/SimRobotHelp/Help/interface/helpmenu.html ../Src/SimRobotHelp/Help/interface/interface.html ../Src/SimRobotHelp/Help/interface/menus.html ../Src/SimRobotHelp/Help/interface/objectwindow.html ../Src/SimRobotHelp/Help/interface/sensorwindow.html ../Src/SimRobotHelp/Help/interface/simulationmenu.html ../Src/SimRobotHelp/Help/interface/statusbar.html ../Src/SimRobotHelp/Help/interface/toolbar.html ../Src/SimRobotHelp/Help/interface/treewindow.html ../Src/SimRobotHelp/Help/interface/viewmenu.html ../Src/SimRobotHelp/Help/interface/windowmenu.html ../Src/SimRobotHelp/Help/rosiml/rosi.html ../Src/SimRobotHelp/Help/rosiml/rosi_controller.html ../Src/SimRobotHelp/Help/simrobothelp.css | prebuild
	@echo "help.qhp (qhelpgenerator)"
	@qhelpgenerator ../Src/SimRobotHelp/Help/help.qhp -o "../Build/SimRobotHelp/Help/help.qch"

../Build/SimRobotHelp/Help/helpcollection.qhc: ../Src/SimRobotHelp/Help/helpcollection.qhcp ../Build/SimRobotHelp/Help/help.qch | prebuild
	@echo "helpcollection.qhcp (qcollectiongenerator)"
	@qcollectiongenerator ../Src/SimRobotHelp/Help/helpcollection.qhcp -o "../Build/SimRobotHelp/Help/helpcollection.qhc"

$(INTDIR)/moc_HelpWidget.cpp: ../Src/SimRobotHelp/HelpWidget.h  | prebuild
	@echo "HelpWidget.h (Qt moc)"
	@moc-qt4 -DLINUX -DQT_NO_DEBUG -DQT_SHARED -DQT_GUI_LIB -DQT_CORE_LIB -DQT_HELP_LIB -DDEBUG -D_DEBUG -I../Src/SimRobotHelp -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4/QtHelp -I/usr/include/qt4 ../Src/SimRobotHelp/HelpWidget.h -o $(INTDIR)/moc_HelpWidget.cpp

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

PROJECTNAME := SimRobotHelp
CONFIG := Release
OUTDIR := $(SOLUTIONDIR)/../Build/SimRobotHelp/Linux/$(CONFIG)
INTDIR := $(SOLUTIONDIR)/../Build/SimRobotHelp/Linux/$(CONFIG)
TARGET := $(OUTDIR)/libSimRobotHelp.so
OBJECTS := \
	$(INTDIR)/moc_HelpWidget.o\
	$(INTDIR)/qrc_SimRobotHelp.o\
	$(INTDIR)/HelpModule.o\
	$(INTDIR)/HelpWidget.o\

PCHOBJECT := 
CUSTOMOBJECTS := \
	$(INTDIR)/qrc_SimRobotHelp.cpp\
	../Build/SimRobotHelp/Help/help.qch\
	../Build/SimRobotHelp/Help/helpcollection.qhc\
	$(INTDIR)/moc_HelpWidget.cpp\

DEPENDENCIES := 

DEFINES := -D"LINUX" -D"QT_NO_DEBUG" -D"QT_SHARED" -D"QT_GUI_LIB" -D"QT_CORE_LIB" -D"QT_HELP_LIB" -D"NDEBUG"
INCPATHS := -I"$(INTDIR)" -I"../Src/SimRobotHelp" -I"/usr/include/qt4/QtCore" -I"/usr/include/qt4/QtGui" -I"/usr/include/qt4/QtHelp" -I"/usr/include/qt4"
LIBPATHS := 
LIBS := -lrt -lpthread -lQtGui -lQtCore -lQtHelp

BUILDFLAGS := -MMD $(DEFINES) $(INCPATHS)   -fpic -pipe -Wall -W -Wno-unused-parameter -Wno-non-virtual-dtor -Wno-deprecated -Wno-ignored-qualifiers -Wno-unused -O3 -fomit-frame-pointer
LINKFLAGS :=  -shared -fpic -s $(LIBPATHS) $(LIBS)

.PHONY: all prebuild clean dist distclean

all: $(TARGET)

$(TARGET): $(OBJECTS) $(CUSTOMOBJECTS) $(DEPENDENCIES)
	@echo Linking...
	@$(CXX) -o $@ $(OBJECTS) $(LDFLAGS) $(LINKFLAGS) 

$(INTDIR)/moc_HelpWidget.o: $(INTDIR)/moc_HelpWidget.cpp  | prebuild
	@echo "moc_HelpWidget.cpp"
	@$(CXX) $(CXXFLAGS) $(BUILDFLAGS) -o $@ -c $<

$(INTDIR)/qrc_SimRobotHelp.o: $(INTDIR)/qrc_SimRobotHelp.cpp  | prebuild
	@echo "qrc_SimRobotHelp.cpp"
	@$(CXX) $(CXXFLAGS) $(BUILDFLAGS) -o $@ -c $<

$(INTDIR)/HelpModule.o: ../Src/SimRobotHelp/HelpModule.cpp  | prebuild
	@echo "HelpModule.cpp"
	@$(CXX) $(CXXFLAGS) $(BUILDFLAGS) -o $@ -c $<

$(INTDIR)/HelpWidget.o: ../Src/SimRobotHelp/HelpWidget.cpp  | prebuild
	@echo "HelpWidget.cpp"
	@$(CXX) $(CXXFLAGS) $(BUILDFLAGS) -o $@ -c $<

$(INTDIR)/qrc_SimRobotHelp.cpp: ../Src/SimRobotHelp/SimRobotHelp.qrc ../Src/SimRobotHelp/Help/Icons/SimRobot.png ../Src/SimRobotHelp/Help/Icons/app_exit.bmp ../Src/SimRobotHelp/Help/Icons/app_exit_xp.bmp ../Src/SimRobotHelp/Help/Icons/architecture.png ../Src/SimRobotHelp/Help/Icons/crystal_toolbar.bmp ../Src/SimRobotHelp/Help/Icons/status_bar.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_camera.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_clock.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_color_image.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_column_graph.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_copy.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_cut.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_drag_and_drop_plane.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_grid.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_help.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_light.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_line_graph.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_monochrome_image.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_motion_blur.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_new.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_open.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_paste.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_reset.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_save.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_search.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_shader_interface.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_show_sensors.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_start.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_step.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_stereogram.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_surface_rendering.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_tree_view.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_update_view.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_view_item.bmp ../Src/SimRobotHelp/Help/api/api.html ../Src/SimRobotHelp/Help/basics/architecture.html ../Src/SimRobotHelp/Help/basics/basics.html ../Src/SimRobotHelp/Help/basics/physics.html ../Src/SimRobotHelp/Help/basics/quickstart.html ../Src/SimRobotHelp/Help/basics/sensing.html ../Src/SimRobotHelp/Help/basics/specification.html ../Src/SimRobotHelp/Help/console/commands.html ../Src/SimRobotHelp/Help/examples/examples.html ../Src/SimRobotHelp/Help/faq/faq.html ../Src/SimRobotHelp/Help/help.qhp ../Src/SimRobotHelp/Help/helpcollection.qhcp ../Src/SimRobotHelp/Help/how_to_help.html ../Src/SimRobotHelp/Help/index.html ../Src/SimRobotHelp/Help/interface/console.html ../Src/SimRobotHelp/Help/interface/editmenu.html ../Src/SimRobotHelp/Help/interface/editorwindow.html ../Src/SimRobotHelp/Help/interface/filemenu.html ../Src/SimRobotHelp/Help/interface/helpmenu.html ../Src/SimRobotHelp/Help/interface/interface.html ../Src/SimRobotHelp/Help/interface/menus.html ../Src/SimRobotHelp/Help/interface/objectwindow.html ../Src/SimRobotHelp/Help/interface/sensorwindow.html ../Src/SimRobotHelp/Help/interface/simulationmenu.html ../Src/SimRobotHelp/Help/interface/statusbar.html ../Src/SimRobotHelp/Help/interface/toolbar.html ../Src/SimRobotHelp/Help/interface/treewindow.html ../Src/SimRobotHelp/Help/interface/viewmenu.html ../Src/SimRobotHelp/Help/interface/windowmenu.html ../Src/SimRobotHelp/Help/rosiml/rosi.html ../Src/SimRobotHelp/Help/rosiml/rosi_controller.html ../Src/SimRobotHelp/Help/simrobothelp.css ../Src/SimRobotHelp/Icons/back.png ../Src/SimRobotHelp/Icons/forward.png ../Src/SimRobotHelp/Icons/help.png ../Src/SimRobotHelp/Icons/home.png ../Src/SimRobotHelp/Icons/locate.png ../Src/SimRobotHelp/SimRobotHelp.qrc | prebuild
	@echo "SimRobotHelp.qrc (Qt rcc)"
	@rcc -name SimRobotHelp ../Src/SimRobotHelp/SimRobotHelp.qrc -o $(INTDIR)/qrc_SimRobotHelp.cpp

../Build/SimRobotHelp/Help/help.qch: ../Src/SimRobotHelp/Help/help.qhp ../Src/SimRobotHelp/Help/Icons/SimRobot.png ../Src/SimRobotHelp/Help/Icons/app_exit.bmp ../Src/SimRobotHelp/Help/Icons/app_exit_xp.bmp ../Src/SimRobotHelp/Help/Icons/architecture.png ../Src/SimRobotHelp/Help/Icons/crystal_toolbar.bmp ../Src/SimRobotHelp/Help/Icons/status_bar.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_camera.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_clock.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_color_image.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_column_graph.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_copy.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_cut.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_drag_and_drop_plane.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_grid.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_help.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_light.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_line_graph.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_monochrome_image.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_motion_blur.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_new.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_open.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_paste.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_reset.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_save.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_search.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_shader_interface.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_show_sensors.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_start.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_step.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_stereogram.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_surface_rendering.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_tree_view.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_update_view.bmp ../Src/SimRobotHelp/Help/Icons/toolbar_view_item.bmp ../Src/SimRobotHelp/Help/api/api.html ../Src/SimRobotHelp/Help/basics/architecture.html ../Src/SimRobotHelp/Help/basics/basics.html ../Src/SimRobotHelp/Help/basics/physics.html ../Src/SimRobotHelp/Help/basics/quickstart.html ../Src/SimRobotHelp/Help/basics/sensing.html ../Src/SimRobotHelp/Help/basics/specification.html ../Src/SimRobotHelp/Help/console/commands.html ../Src/SimRobotHelp/Help/examples/examples.html ../Src/SimRobotHelp/Help/faq/faq.html ../Src/SimRobotHelp/Help/help.qhp ../Src/SimRobotHelp/Help/helpcollection.qhcp ../Src/SimRobotHelp/Help/how_to_help.html ../Src/SimRobotHelp/Help/index.html ../Src/SimRobotHelp/Help/interface/console.html ../Src/SimRobotHelp/Help/interface/editmenu.html ../Src/SimRobotHelp/Help/interface/editorwindow.html ../Src/SimRobotHelp/Help/interface/filemenu.html ../Src/SimRobotHelp/Help/interface/helpmenu.html ../Src/SimRobotHelp/Help/interface/interface.html ../Src/SimRobotHelp/Help/interface/menus.html ../Src/SimRobotHelp/Help/interface/objectwindow.html ../Src/SimRobotHelp/Help/interface/sensorwindow.html ../Src/SimRobotHelp/Help/interface/simulationmenu.html ../Src/SimRobotHelp/Help/interface/statusbar.html ../Src/SimRobotHelp/Help/interface/toolbar.html ../Src/SimRobotHelp/Help/interface/treewindow.html ../Src/SimRobotHelp/Help/interface/viewmenu.html ../Src/SimRobotHelp/Help/interface/windowmenu.html ../Src/SimRobotHelp/Help/rosiml/rosi.html ../Src/SimRobotHelp/Help/rosiml/rosi_controller.html ../Src/SimRobotHelp/Help/simrobothelp.css | prebuild
	@echo "help.qhp (qhelpgenerator)"
	@qhelpgenerator ../Src/SimRobotHelp/Help/help.qhp -o "../Build/SimRobotHelp/Help/help.qch"

../Build/SimRobotHelp/Help/helpcollection.qhc: ../Src/SimRobotHelp/Help/helpcollection.qhcp ../Build/SimRobotHelp/Help/help.qch | prebuild
	@echo "helpcollection.qhcp (qcollectiongenerator)"
	@qcollectiongenerator ../Src/SimRobotHelp/Help/helpcollection.qhcp -o "../Build/SimRobotHelp/Help/helpcollection.qhc"

$(INTDIR)/moc_HelpWidget.cpp: ../Src/SimRobotHelp/HelpWidget.h  | prebuild
	@echo "HelpWidget.h (Qt moc)"
	@moc-qt4 -DLINUX -DQT_NO_DEBUG -DQT_SHARED -DQT_GUI_LIB -DQT_CORE_LIB -DQT_HELP_LIB -DNDEBUG -I../Src/SimRobotHelp -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4/QtHelp -I/usr/include/qt4 ../Src/SimRobotHelp/HelpWidget.h -o $(INTDIR)/moc_HelpWidget.cpp

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

