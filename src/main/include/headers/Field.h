#ifndef _MATCH_FIELD_H
#define _MATCH_FIELD_H

#include "headers/Libraries.h"

class Field2d {
	private:
		inline static Field2d* instance{nullptr};
		Field2d() = default;
		~Field2d() = default;
		
	public:
		Field2d(const Field2d&) = delete;
		Field2d& operator=(const Field2d&) = delete;

		static Field2d* GetInstance(){
			if ( !instance ){
				instance = new Field2d();
			}
			return instance;
		}

		frc::Field2d m_field;
};

#endif	// _MATCH_FIELD_H