#include "stdafx.h"
#include "Form1.h"
#include "serialcom.h"
#include "calculations.h"

int NUM_CYCLES;
int MOST_CYCLES;
double PB1, PB2, PB3;



namespace GUI_1{
	

	Form1::Form1(void)
	{
		InitializeComponent();
		//
		//TODO: Add the constructor code here
		//
		this -> final_values = gcnew array<double>(9);
		this -> increments = gcnew array<double>(9);
		this -> angles_to_AVR = gcnew array<double>(9);
		this -> read_values = gcnew array<double>(18);
		this -> FLAG_NEW_VALUES = FALSE;
		this -> flags_new_value = gcnew array<double>(9);
		this -> trSpeed1 -> Value = 100;
		this -> trSpeed2 -> Value = 100;
		this -> trSpeed3 -> Value = 100;
		this -> timer2 -> Enabled = false;
		this -> timer3 -> Enabled = false;
		for (int i = 0; i < 9; i++)
		{
			angles_to_AVR[i] = 90.0f;
		}
	}

	void Form1::chlabel2(float num)
	{
		label2 -> Text = " " + num;
	}
	System::Windows::Forms::Panel ^ Form1::returnpanel(){
		return this->panelgl;
	}
	System::Windows::Forms::Label ^ Form1::returnlabel2(){
		return label2;
	}
	System::Timers::Timer ^ Form1::returntimer4()
	{
		return timer4;
	}
	void Form1::updateTrackBar(int vertex)
	{
		updateTrackBars(vertex);
	}

int Form1::updateHand(int source){

			double *data_angles = new double[9];
			SerialCommunication comm1;
			// Check source: 1 = GUI, 2 = Emergency Button
			if (source == 1){ // GUI
				for (int i = 0; i < 9; i++)
				{
					data_angles[i] = angles_to_AVR[i];
				}

				unsigned int * data = _produce_counts_all(data_angles);
				data[18] = 'B';
				comm1.SerialWrite(data);

				delete[] data;
				delete[] data_angles;
			}
			else if (source == 2){
				unsigned int * data = new unsigned int[19];
				for (int i = 0; i < 19; i++)
				{
					data[i] = 'R';
				}
				comm1.SerialWrite(data);

				delete[] data;
				delete[] data_angles;
			}
			else {
			 // Get data from OpenGL class
			}
			return 1;
		}
void Form1::resetHand(){
			trFinger1Ph1 -> Value = 900;
			trFinger1Ph2 -> Value = 900;
			trFinger1Ph3 -> Value = 900;
			trFinger2Ph1 -> Value = 900;
			trFinger2Ph2 -> Value = 900;
			trFinger2Ph3 -> Value = 900;
			trFinger3Ph1 -> Value = 900;
			trFinger3Ph2 -> Value = 900;
			trFinger3Ph3 -> Value = 900;

			this -> lblFinger1Phalanx1 -> Text = "90.0";
			this -> lblFinger1Phalanx2 -> Text = "90.0";
			this -> lblFinger1Phalanx3 -> Text = "90.0";
			this -> lblFinger2Phalanx1 -> Text = "90.0";
			this -> lblFinger2Phalanx2 -> Text = "90.0";
			this -> lblFinger2Phalanx3 -> Text = "90.0";
			this -> lblFinger3Phalanx1 -> Text = "90.0";
			this -> lblFinger3Phalanx2 -> Text = "90.0";
			this -> lblFinger3Phalanx3 -> Text = "90.0";

			o_gl -> update_model_angles(0, 90.0f);
			o_gl -> update_model_angles(1, 90.0f);
			o_gl -> update_model_angles(2, 90.0f);
			o_gl -> update_model_angles(4, 90.0f);
			o_gl -> update_model_angles(5, 90.0f);
			o_gl -> update_model_angles(6, 90.0f);
			o_gl -> update_model_angles(8, 90.0f);
			o_gl -> update_model_angles(9, 90.0f);
			o_gl -> update_model_angles(10, 90.0f);

			for (int i = 0; i < 9; i++)
			{
				angles_to_AVR[i] = 90.0f;
			}
			
			int i = updateHand(1);
}

}
