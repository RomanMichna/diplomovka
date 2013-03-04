/**
* @file ModuleManager.cpp
* Implementation of a class representing the module manager.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "ModuleManager.h"
#include "Platform/BHAssert.h"
#include "Tools/Settings.h"
#include "Tools/Streams/InStreams.h"
#include <algorithm>

DefaultModule::DefaultModule() :
  ModuleBase("default", "Infrastructure")
{
  // base class will add this module to the list of all modules, but it should not
  // so remove it again
  modules.pop_back();
}

void DefaultModule::setRepresentations(const std::list<ModuleManager::ModuleState>& modules)
{
  for(std::list<ModuleManager::ModuleState>::const_iterator i = modules.begin(); i != modules.end(); ++i)
    for(Representations::List::const_iterator j = i->module->representations.begin(); j != i->module->representations.end(); ++j)
      if(std::find(representations.begin(), representations.end(), j->name) == representations.end())
        representations.push_back(*j);
}

ModuleManager::ModuleManager() :
  timeStamp(0),
  defaultModule(new DefaultModule)
{
  for(std::list<ModuleBase*>::iterator i = ModuleBase::modules.begin(); i != ModuleBase::modules.end(); ++i)
    modules.push_back(ModuleState(*i));
  defaultModule->setRepresentations(modules);
  modules.push_back(ModuleState(defaultModule));
}

ModuleManager::ModuleManager(const std::string& process) :
  timeStamp(0),
  defaultModule(new DefaultModule)
{
  ConfigMap cm;
  if(cm.read("Processes/" + process + "Modules.cfg", true) >= 0)
  {
    cm.setFlags(cm.getFlags() | ConfigMap::READONLY);
    std::vector<std::string> filter;
    cm["modules"] >> filter;
    for(std::list<ModuleBase*>::iterator i = ModuleBase::modules.begin(); i != ModuleBase::modules.end(); ++i)
      if(std::find(filter.begin(), filter.end(), (*i)->name) != filter.end())
        modules.push_back(ModuleState(*i));
    defaultModule->setRepresentations(modules);
    modules.push_back(ModuleState(defaultModule));
  }
  else
    ASSERT(false);
}

ModuleManager::~ModuleManager()
{
  destroy();
}

void ModuleManager::destroy()
{
  if(defaultModule) // prevent multiple destructions
  {
    ConfigMap cm;
    cm["sharedRepresentations"] << std::vector<char>();
    cm["representationProviders"] << std::vector<char>();
    InConfigMap empty(cm);
    update(empty); // destruct everything
    delete defaultModule;
    defaultModule = 0;
  }
}

void ModuleManager::update(In& stream, unsigned timeStamp)
{
  std::list<Provider> providersToDelete(providers),
                      providersToCreate,
                      providersBackup(providers);
  std::list<Shared> sharedToDelete(shared),
                    sharedToCreate,
                    sharedBackup(shared);

  std::string representation,
              module;

  providers.clear();
  selected.clear();
  shared.clear();

  // Remove all markings
  for(std::list<ModuleState>::iterator j = modules.begin(); j != modules.end(); ++j)
    j->required = false;

  Configuration config;
  stream >> config;

  // fill shared representations
  for(std::vector<std::string>::iterator j = config.sharedRepresentations.begin(); j != config.sharedRepresentations.end(); ++j)
    shared.push_back(*j);

  std::list<std::string> defaultProviders;
  for(std::vector<Configuration::RepresentationProvider>::iterator rp = config.representationProviders.begin(); rp != config.representationProviders.end() || !defaultProviders.empty(); void(rp != config.representationProviders.end() ? ++rp : rp))
  {
    if(rp != config.representationProviders.end())
    {
      if(rp->provider == "default")
      {
        defaultProviders.push_back(rp->representation); // process after all real modules
        continue; // read next
      }
      representation = rp->representation;
      module = rp->provider;
    }
    else
    {
      representation = defaultProviders.front();
      defaultProviders.pop_front();
      module = "default";
    }
    std::list<ModuleState>::iterator j;
    for(j = modules.begin(); j != modules.end(); ++j)
      if(module == j->module->name)
      {
        Representations::List::const_iterator i;
        for(i = j->module->representations.begin(); i != j->module->representations.end(); ++i)
          if(representation == i->name)
          {
            Provider provider(i->name, &*j, i->update, i->create, i->free);
            std::list<Provider>::iterator m = std::find(providers.begin(), providers.end(), provider);
            if(m == providers.end())
            {
              selected[i->name] = j->module->name;
              providers.push_back(provider);
              // Is the representation provided one of the shared ones?
              std::list<Shared>::iterator k = std::find(shared.begin(), shared.end(), representation);
              if(k != shared.end()) // yes
              {
                k->out = i->out; // set write handler
                k->in = 0; // remove read handler
              }
              // This provider may have a requirement that is a shared representation
              for(k = shared.begin(); k != shared.end(); ++k)
                if(!k->out && !k->in) // only search those with unknown state
                {
                  Requirements::List::const_iterator m = std::find(j->module->requirements.begin(), j->module->requirements.end(), k->representation);
                  if(m != j->module->requirements.end()) // yes, one of the requirements is shared
                  {
                    k->create = m->create; // so we know how to create, free, and read it
                    k->free = m->free;
                    k->in = m->in;
                    // Is the representation already constructed
                    std::list<Shared>::iterator n = std::find(sharedToDelete.begin(), sharedToDelete.end(), k->representation);
                    if(n == sharedToDelete.end()) // no, representation is new
                      sharedToCreate.push_back(*k);
                    else // it already exists and is still needed, so don't delete it
                      sharedToDelete.erase(n);
                  }
                }
              // Is the representation already constructed
              m = std::find(providersToDelete.begin(), providersToDelete.end(), provider);
              if(m == providersToDelete.end()) // no, representation is new
                providersToCreate.push_back(providers.back());
              else // it already exists and is still needed, so don't delete it
                providersToDelete.erase(m);
            }
            else if(module != m->moduleState->module->name && module != "default")
            {
              OUTPUT_ERROR("Error: " << representation << " provided by more than one module!");
              providers = providersBackup;
              shared = sharedBackup;
              return;
            }
            break;
          }
        if(module != "default" && i == j->module->representations.end()) // module actually provides representation
        {
          OUTPUT_ERROR("Error: " << module << " does not provide " << representation << "!");
          providers = providersBackup;
          shared = sharedBackup;
          return;
        }
        j->required = true;
        break;
      }

#ifdef MULTI_PROCESS
    // if j == modules.end() then module name is unknown but hopefully valid in another process
#else
    // everything is in a single process, so we know all valid module names
    std::list<ModuleBase*>::iterator k;
    for(k = ModuleBase::modules.begin(); k != ModuleBase::modules.end() && module != (*k)->name; ++k)
      ;
    if(module != "default" && k == ModuleBase::modules.end())
    {
      OUTPUT_ERROR("Error: " << representation << " cannot be provided by unknown module " << module << "!");
      providers = providersBackup;
      shared = sharedBackup;
      return;
    }
#endif
  }

  // Are all shared representations either received of sent?
  std::list<Shared>::const_iterator i;
  for(i = shared.begin(); i != shared.end(); ++i)
    if(!i->out && !i->in)
    {
      OUTPUT_ERROR("Error: shared representation " << i->representation << " neither sent nor received!");
      break;
    }

  if(i != shared.end() || !sortProviders())
  {
    providers = providersBackup;
    shared = sharedBackup;
    return;
  }

  // Delete all modules that are not required anymore
  for(std::list<ModuleState>::iterator j = modules.begin(); j != modules.end(); ++j)
    if(!j->required && j->instance)
    {
      delete j->instance;
      j->instance = 0;
    }

  // Delete all representations that are not required anymore
  for(std::list<Provider>::iterator j = providersToDelete.begin(); j != providersToDelete.end(); ++j)
  {
    // don't delete representations that are shared now
    std::list<Shared>::iterator k = std::find(shared.begin(), shared.end(), j->representation);
    if(k == shared.end() || !k->in)
      j->free();
  }

  // Delete all shared representations that are not required anymore
  for(std::list<Shared>::iterator j = sharedToDelete.begin(); j != sharedToDelete.end(); ++j)
    if(j->in) // don't delete representations that would immediately be recreated by a provider
      if(std::find(providersToCreate.begin(), providersToCreate.end(), j->representation) == providersToCreate.end())
        j->free();

  // Create all shared representations that are missing
  for(std::list<Shared>::iterator j = sharedToCreate.begin(); j != sharedToCreate.end(); ++j)
    if(j->in) // don't create representations that were provided by a provider before
      if(std::find(providersToDelete.begin(), providersToDelete.end(), j->representation) == providersToDelete.end())
        j->create();

  // Create all representations that are missing
  for(std::list<Provider>::iterator j = providersToCreate.begin(); j != providersToCreate.end(); ++j)
  {
    // don't create representations that were shared before
    std::list<Shared>::iterator k = std::find(sharedToDelete.begin(), sharedToDelete.end(), j->representation);
    if(k == sharedToDelete.end() || !k->in)
      j->create();
  }

  // Create all modules that are missing
  for(std::list<ModuleState>::iterator j = modules.begin(); j != modules.end(); ++j)
    if(j->required && !j->instance)
      j->instance = j->module->createNew();

  this->timeStamp = timeStamp;
}

bool ModuleManager::sortProviders()
{
  std::list<std::string> provided; /**< The representation already provided. */

  // Add shared representations that are received by this process
  for(std::list<Shared>::const_iterator i = shared.begin(); i != shared.end(); ++i)
    if(i->in)
      provided.push_back(i->representation);

  int remaining = providers.size(), /**< The number of entries not correct sequence so far. */
      pushBackCount = remaining; /**< The number of push_backs still allowed. If zero, no valid sequence is possible. */
  std::list<Provider>::iterator i = providers.begin();
  while(i != providers.end())
  {
    const Requirements::List& requirements = i->moduleState->module->requirements;
    Requirements::List::const_iterator j;
    for(j = requirements.begin(); j != requirements.end(); ++j)
      if(j->name != i->representation && std::find(provided.begin(), provided.end(), j->name) == provided.end())
        break;
    if(requirements.begin() != requirements.end() && j != requirements.end()) // at least one requirement missing
    {
      if(pushBackCount) // still one left to try
      {
        providers.push_back(*i);
        i = providers.erase(i);
        --pushBackCount;
      }
      else // we checked all, none was satisfied
      {
        std::string text;
        for(std::list<std::string>::const_iterator k = provided.begin(); k != provided.end(); ++k)
        {
          text += text == "" ? "" : ", ";
          text += *k;
        }
        std::string text2 = "";
        while(i != providers.end())
        {
          text2 += text2 == "" ? "" : ", ";
          text2 += i->representation;
          ++i;
        }
        if(text == "")
        {
          OUTPUT_ERROR("Error: requirements missing for providers for " << text2 << ".");
        }
        else
        {
          OUTPUT_ERROR("Error: only found consistent providers for " << text <<
                       ".\nRequirements missing for providers for " << text2 << ".");
        }
        return false;
      }
    }
    else // we found one with all requirements fulfilled
    {
      provided.push_back(i->representation); // add representation provided
      ++i; // continue with next provider
      --remaining; // we have one less to go,
      pushBackCount = remaining; // and the search starts again
    }
  }
  return true;
}

void ModuleManager::load()
{
  InConfigMap stream(Global::getSettings().expandLocationFilename("modules.cfg"));
  if(!stream.exists())
  {
    OUTPUT_ERROR("Failed to load modules.cfg correctly.");
    ASSERT(true); // since when modules aren't loaded correctly ther come up other failures
  }
  update(stream);
}

void ModuleManager::execute()
{
  // Execute all providers in the given sequence
  for(std::list<Provider>::iterator i = providers.begin(); i != providers.end(); ++i)
    if(i->moduleState->instance)
    {
#ifdef TARGET_ROBOT
      unsigned timeStamp = SystemCall::getCurrentSystemTime();
#endif
      i->update(*i->moduleState->instance);
#ifdef TARGET_ROBOT
      int duration = SystemCall::getTimeSince(timeStamp);
      if(duration > 100 &&
         ((!Global::getDebugRequestTable().isActive("representation:JPEGImage") &&
           !Global::getDebugRequestTable().isActive("representation:Image")) ||
          duration > 500))
        TRACE("TIMING: providing %s took %d ms at %d s after start",
              i->representation.c_str(), duration, timeStamp / 1000 - 10);
#endif
    }
  BH_TRACE;

  DEBUG_RESPONSE("automated requests:ModuleTable",
  {
    Global::getDebugOut().bin << (unsigned) modules.size();
    for(std::list<ModuleState>::const_iterator i = modules.begin(); i != modules.end(); ++i)
    {
      Global::getDebugOut().bin << i->module->name << i->module->category;
      const Requirements::List& requirements = i->module->requirements;
      Global::getDebugOut().bin << (unsigned) requirements.size();
      for(Requirements::List::const_iterator j = requirements.begin(); j != requirements.end(); ++j)
        Global::getDebugOut().bin << j->name;
      const Representations::List& representations = i->module->representations;
      Global::getDebugOut().bin << (unsigned) representations.size();
      for(Representations::List::const_iterator j = representations.begin(); j != representations.end(); ++j)
        Global::getDebugOut().bin << j->name;
    }
    Global::getDebugOut().bin << (unsigned) selected.size();
    for(std::map<const char*, const char*>::const_iterator i = selected.begin(); i != selected.end(); ++i)
      Global::getDebugOut().bin << i->first << i->second;
    Global::getDebugOut().finishMessage(idModuleTable);
  });
}

void ModuleManager::readPackage(In& stream)
{
  unsigned timeStamp;
  stream >> timeStamp;
  // Communication is only possible if both sides are based on the same module request.
  if(timeStamp == this->timeStamp)
  {
    for(std::list<Shared>::const_iterator i = shared.begin(); i != shared.end(); ++i)
      if(i->in)
        i->in(stream);
  }
  else
    stream.skip(10000000); // skip everything
}

void ModuleManager::writePackage(Out& stream) const
{
  stream << timeStamp;
  for(std::list<Shared>::const_iterator i = shared.begin(); i != shared.end(); ++i)
    if(i->out)
      i->out(stream);
}
