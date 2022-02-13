//#include "openmorph_pt.h"
//
//OpenMorph::OpenMorph() : controllerId(0) {
//  stopRequested = false;
//  dir = MorphDir::dec;
//}
///*
//void OpenMorph::registerIOpenMorphCb(IOpenMorph *p_pIOpenMorphCb) {
//  m_pIOpenMorphCb = p_pIOpenMorphCb;
//}
//*/
//void OpenMorph::reset() {
//  if(inProgress) {
//    stopRequested = true;
//    delay(1.5 * (morphTime / 128.0));
//    inProgress = false;
//  }
//  dir = MorphDir::dec;
//}
//
//void OpenMorph::exec(MorphDir dir, uint16_t morph_time) {
//
//  if(dir != MorphDir::inc && dir != MorphDir::dec)
//    return;
//  
//  this->reset();
//  
//  this->dir = dir;
//  this->morphTime = morph_time;
//  
//  this->inProgress = true; // activates the thread
//}
//
//int OpenMorph::do_morph(struct OpenMorph *pt)
//{
//  static uint16_t delayTime = pt->morphTime / 128.0;
//  static uint16_t lastMorphUpdate = 0;
//  static uint8_t i;
//
//  static uint8_t start_value = 0;
//  static uint8_t stop_value = 127;
//  static uint8_t current_value;
//  static int8_t increment = 1;
//
//  static unsigned long morphTime = pt->morphTime;
//  PT_BEGIN(pt);
//
//  while (1) {
//    PT_WAIT_UNTIL(pt, true == pt->inProgress);
//
//    if (pt->dir == MorphDir::dec) {
//      start_value = 127;
//      stop_value = 0;
//      increment = -1;
//      //morphTime = HX_MORPH_0_MORPH_DEC_TIME_MS;
//    }
//    else {
//      start_value = 0;
//      stop_value = 127;
//      increment = 1;
//      //morphTime = HX_MORPH_0_MORPH_INC_TIME_MS;
//    }
//    morphTime = pt->morphTime;
//    delayTime = morphTime / 128.0;
//    
//    if(pt->m_pIOpenMorphCb)
//      pt->m_pIOpenMorphCb->onMorphBegin(pt->controllerId, pt->dir, start_value, stop_value);
//      
//    if (morphTime == 0) {
//      if(pt->m_pIOpenMorphCb)
//        pt->m_pIOpenMorphCb->onMorphUpdate(pt->controllerId, stop_value);
//      pt->inProgress = false;
//    }
//    else {
//      current_value = start_value;
//
//      while (current_value != stop_value) {
//        if (pt->stopRequested) {
//          if(pt->m_pIOpenMorphCb)
//            pt->m_pIOpenMorphCb->onMorphUpdate(pt->controllerId, stop_value);
//          if(pt->m_pIOpenMorphCb)
//            pt->m_pIOpenMorphCb->onMorphEnd(pt->controllerId, pt->dir);
//          pt->stopRequested = false;
//          break;
//        }
//        lastMorphUpdate = millis();
//
//        if(pt->m_pIOpenMorphCb)
//          pt->m_pIOpenMorphCb->onMorphUpdate(pt->controllerId, current_value);
//        
//        PT_WAIT_UNTIL(pt, millis() - lastMorphUpdate > delayTime);
//        current_value += increment;
//      }
//      if(pt->m_pIOpenMorphCb)
//        pt->m_pIOpenMorphCb->onMorphUpdate(pt->controllerId, stop_value);
//    }
//    pt->inProgress = false;
//  }
//  PT_END(pt);
//}
