/**
 * A tool that reads data from a shared memory provided by the SSL vision server
 * and broadcasts it via team communication.
 * @author <a href="mailto:afabisch@tzi.de">Alexander Fabisch</a>
 */
#include "SSLConnector/Framework.h"
#include "SSLConnector/DebugHandler.h"
#include "SSLConnector/TeamCommWrapper.h"

#include "Modules/Modeling/GroundTruthProvider/SSLVisionProvider.h"

#include "Tools/Team.h"
#include "Platform/BHAssert.h"
#include <csignal>
#include <iostream>

bool run = true;

static void sighandlerShutdown(int sig)
{
  run = false;
  Framework::destroy("SSLConnector"); // TODO clean up
  std::cerr << "Stopped." << std::endl;
}

int main()
{
  Framework& framework = *Framework::getInstance("SSLConnector", 12);
  DebugHandler debugHandler(framework.debugOut);
  TeamCommWrapper tcw(framework.teamOut, framework.settings.teamPort);
  signal(SIGTERM, sighandlerShutdown);
  signal(SIGINT, sighandlerShutdown);

  while(run && !framework.requestShutdown)
  {
    tcw.receive();
    TEAM_OUTPUT(idRobot, bin, framework.playerNumber);
    tcw.send();
    framework.moduleManager.execute();
    debugHandler.frameFinished();
    usleep(100000); // 100 ms
  }
  Framework::destroy("SSLConnector");
}
