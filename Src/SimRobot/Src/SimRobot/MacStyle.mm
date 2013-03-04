#include "MacStyle.h" 
#include "MainWindow.h"

void MacStyle::addFullscreen(MainWindow *window)
{
  NSString *string = [NSString string];

  if([string respondsToSelector:@selector(linguisticTagsInRange:scheme:options:orthography:tokenRanges:)])
  {
    NSView* nsview = (NSView*) window->winId();
    NSWindow* nswindow = [nsview window];
    [nswindow setCollectionBehavior:NSWindowCollectionBehaviorFullScreenPrimary];
  }
}
