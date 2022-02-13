//#ifndef _open_morph_pt_h_
//#define _open_morph_pt_h_
//  
//#include <pt.h>
//#include <Arduino.h>
//
//enum MorphDir {
//  inc = 0,
//  dec = 1
//};
//
//struct IOpenMorph {
//  virtual void onMorphBegin(uint8_t controlId, MorphDir dir, uint8_t startValue, uint8_t stopValue)=0;
//  virtual void onMorphUpdate(uint8_t controlId, uint8_t currentValue)=0;
//  virtual void onMorphEnd(uint8_t controlId, MorphDir dir)=0;
//};
//
//class OpenMorph : public pt {
//  public:
//    OpenMorph();
//    //void registerIOpenMorphCb(IOpenMorph *p_pIOpenMorphCb);
//    bool stopRequested = false;
//    bool inProgress = false;
//    uint8_t controllerId = 0;
//    MorphDir dir;
//    uint16_t morphTime;
//    IOpenMorph *m_pIOpenMorphCb = NULL;
//    void reset();
//    void exec(MorphDir dir, uint16_t morph_time);
//    static int do_morph(struct OpenMorph *pt);
//};
//
//
//#endif // _open_morph_pt_h_
