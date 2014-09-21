// -*- C++ -*-
#ifndef JOJO_PDSERVO_H
#define JOJO_PDSERVO_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <cnoid/Body>
#include<cnoid/BodyLoader>
#include<cnoid/VRMLBodyLoader>
//#include <hrpModel/Body.h>
#include "VectorConvert.h"



using namespace RTC;
//using namespace cnoid;

class JojoPDservo : public DataFlowComponentBase
{
public:
  JojoPDservo(RTC::Manager* manager);
  ~JojoPDservo();
  
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
protected:
  TimedDoubleSeq m_qRef;
  InPort<TimedDoubleSeq> m_qRefIn;

  TimedDoubleSeq m_qCur;
  InPort<TimedDoubleSeq> m_qCurIn;

  TimedDoubleSeq m_tauRef;
  OutPort<TimedDoubleSeq> m_tauRefOut;


private:
  double m_dt;
  std::string m_instanceName;

  cnoid::BodyPtr m_robot;
  
  std::vector<double> m_pGain;
  std::vector<double> m_dGain;
  std::vector<double> m_Limit;
  

  bool m_isFirstLoop;
  std::vector<double> m_qPre;
  std::vector<double> m_refPre;
  //std::vector<double> m_qVel;
};


extern "C"
{
  DLL_EXPORT void JojoPDservoInit(RTC::Manager* manager);
};


#endif
