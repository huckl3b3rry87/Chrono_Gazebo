#ifndef SRC_GCVEHICLE_HH_
#define SRC_GCVEHICLE_HH_

class GcVehicle {
public:

	virtual void Synchronize(double time) {};
	virtual void Advance(double step) {};
	virtual bool Init() { return true; }

	virtual int GetId() const { return -1; }

	virtual ~GcVehicle() {}

};

#endif /* SRC_GCVEHICLE_HH_ */
