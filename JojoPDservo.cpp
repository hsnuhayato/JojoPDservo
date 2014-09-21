// -*- C++ -*-
#include "JojoPDservo.h"

//#include <rtm/CorbaNaming.h>
//#include <hrpModel/ModelLoaderUtil.h>
using namespace std;

// Module specification
static const char* JojoPDservo_spec[] =
  {
    "implementation_id", "JojoPDservo",
    "type_name",         "JojoPDservo",
    "description",       "Sequence InPort component",
    "version",           "1.0",
    "vendor",            "R. Kishibe",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "SequenceInComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };



JojoPDservo::JojoPDservo(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_qRefIn("qRefIn", m_qRef),
    m_qCurIn("qCurIn", m_qCur),
    m_tauRefOut("tauRefOut", m_tauRef)
{

}


JojoPDservo::~JojoPDservo()
{

}


RTC::ReturnCode_t JojoPDservo::onInitialize()
{
  //
  // Registration: InPort/OutPort/Service
  //
  // Set InPort buffers
  addInPort("qRefIn", m_qRefIn);
  addInPort("qCurIn", m_qCurIn);
  
  // Set OutPort buffer
  addOutPort("tauRefOut", m_tauRefOut);




  //
  // get properties
  //
  RTC::Properties& prop = getProperties();

  
  coil::stringTo(m_dt, prop["pdservo.dt"].c_str());
  m_instanceName = prop["instance_name"];
  //check if dt==0.001
  std::cout << m_instanceName << ": dt = " << m_dt << std::endl;
  
  //
  // setup robot model
  //
  cnoid::BodyLoader bl;
  m_robot=bl.load( prop["model"].c_str());
  

  //
  // set PD gain
  //
  coil::stringTo(m_pGain, prop["pdservo.pgain"].c_str());
  coil::stringTo(m_dGain, prop["pdservo.dgain"].c_str());
  coil::stringTo(m_Limit, prop["pdservo.torqueLimit"].c_str());

  int dof = m_robot->numJoints();
  //int dof = 32;
  m_qCur.data.length(dof);
  m_qRef.data.length(dof);
  m_tauRef.data.length(dof);
  
  if( m_pGain.size() != dof  ||  m_dGain.size() != dof ) {
    std::cerr << m_instanceName << " : failed to load gain" << std::endl;
  }
  else {
    std::cout << m_instanceName << " : p gain =";  for(int i = 0; i < m_pGain.size(); i++)  std::cout << "  " << m_pGain[i];  std::cout << std::endl;
    std::cout << m_instanceName << " : d gain =";  for(int i = 0; i < m_dGain.size(); i++)  std::cout << "  " << m_dGain[i];  std::cout << std::endl;
    std::cout << m_instanceName << " : d limi =";  for(int i = 0; i < m_dGain.size(); i++)  std::cout << "  " << m_Limit[i];  std::cout << std::endl;

    m_qPre.resize(dof);
    m_refPre.resize(dof);

    //initialize
    for(int i = 0; i < dof; i++) {
      m_qPre[i] = 0.0;
      m_refPre[i]=0.0;
      m_qCur.data[i]=0.0;
      m_qRef.data[i]=0.0;
      m_tauRef.data[i]=0.0;
    }
    m_isFirstLoop = true;
  }
 
  return RTC::RTC_OK;
}


RTC::ReturnCode_t JojoPDservo::onExecute(RTC::UniqueId ec_id)
{
  if( m_qCurIn.isNew() ){
    m_qCurIn.read();
    
    if( m_qRefIn.isNew() )
      m_qRefIn.read();
    
  
    for(int i = 0; i < m_qCur.data.length(); i++) {
      double dq_cur = (m_qCur.data[i] - m_qPre[i] )/m_dt; 
      double dq_ref= (m_qRef.data[i] - m_refPre[i] )/m_dt;
      double diff = m_qRef.data[i] - m_qCur.data[i];
  
      m_tauRef.data[i] = m_pGain[i] * diff + m_dGain[i] * (dq_ref - dq_cur);
      
      //limit 
      if(fabs(m_tauRef.data[i])>m_Limit[i]){
	if(m_tauRef.data[i]>0)
	  m_tauRef.data[i]=m_Limit[i];
	if(m_tauRef.data[i]<0)
	  m_tauRef.data[i]=-m_Limit[i];
      }
      
      m_qPre[i] = m_qCur.data[i];
      m_refPre[i]= m_qRef.data[i];
     
    }//for loop
      
    m_tauRefOut.write();
  }//qCur isnew


  return RTC::RTC_OK;
}


RTC::ReturnCode_t JojoPDservo::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << "on Deactivated" << std::endl;
  return RTC::RTC_OK;
}
//-----------------------------------------------------------------------

extern "C"
{
 
  void JojoPDservoInit(RTC::Manager* manager)
  {
    coil::Properties profile(JojoPDservo_spec);
    manager->registerFactory(profile,
                             RTC::Create<JojoPDservo>,
                             RTC::Delete<JojoPDservo>);
  }
  
};
