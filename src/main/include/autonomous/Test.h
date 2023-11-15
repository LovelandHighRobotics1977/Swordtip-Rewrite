#include "headers/Headers.h"

class BestAutoRoutine{
	public:
        BestAutoRoutine(std::string _name, frc::DriverStation::Alliance _alliance, frc2::CommandPtr* _routine);

		std::string GetName();
		frc::DriverStation::Alliance GetAlliance();
        frc2::Command* GetRoutine();

	private:
        std::string Name;
        frc::DriverStation::Alliance Alliance;
        frc2::CommandPtr* Routine;

};

BestAutoRoutine::BestAutoRoutine(std::string _name, frc::DriverStation::Alliance _alliance, frc2::CommandPtr* _routine){
    Name = _name;
    Alliance = _alliance;
    Routine = _routine;
};

frc::DriverStation::Alliance BestAutoRoutine::GetAlliance(){return Alliance;}
std::string BestAutoRoutine::GetName(){return Name;}
frc2::Command* BestAutoRoutine::GetRoutine(){return Routine->get();}