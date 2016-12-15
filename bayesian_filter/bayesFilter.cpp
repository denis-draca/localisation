#include <iostream>
#include <vector>
#include <string.h>
#include "math.h"


using namespace std;

/* motion probabilities are set as global variables
 * motionMoveProb: probability of moving one step forward
 * 		0.7 ---> move one step forward
 * 		0.3 ---> move two steps forward
 * motionStayProb: probability of keeping the robot where it is
 * 		1.0 ---> do not move
 * 		0.0 ---> robot is moved */
double motionMoveProb[] = { 0.7, 0.3 };
double motionStayProb[] = { 1.0, 0.0 };

/* observation probability given a door or not a door
 * 		0.8 ---> observe a door given there is a door
 * 		0.2 ---> observe no door given there is a door
 * 		0.7 ---> observe no door given there is no door
 * 		0.3 ---> observe a door given there is no door*/
double obsDoorProb[] = { 0.8, 0.2 };
double obsNotDoorProb[] = { 0.7, 0.3 };





//-----------------USEFUL FUNCTIONS-----------------//
//get Array size
template<typename T, int size>
int arraysize(T(&)[size]){return size;}

//Prints an Array
template<typename A> void printarray (A arg[],int arraylength) {
	for ( int n = 0; n < arraylength; n ++)
		cout << arg[n] << ", ";
	cout << "\n";
}

//Prints an Vector Array
template<typename A> void printvector(vector<A> arg) {
	for (int n = 0; n < arg.size(); n ++)
		cout << arg[n] << ", ";
	cout << "\n";
}

//Vector Sum Function
template<typename A> double vectorsum(vector<A> arg) {
  double sum = 0;
  for (int n = 0; n < arg.size(); n ++)
  	sum += arg[n];
  return sum;
}
//---------------------------------------------------//






/* Calculate motion probability
 * 		prePosiiton ---> the previous robot position
 * 		curPosition ---> the current robot position
 * 		control     ---> input control method: staying or moving */
double motionProb( int prePosition, int curPosition, bool control ) {
	double prob = 0;
	/* QUESTION 3 HERE */

    if (control)
    {
        if(curPosition - prePosition == 1)
        {
            prob = motionMoveProb[0];
        }
        else if(curPosition - prePosition == 2)
        {
            prob = motionMoveProb[1];
        }
        else
        {
            prob = 0;
        }
    }
    else
    {
        if (curPosition == prePosition)
        {
            prob = motionStayProb[0];
        }
        else
        {
            prob = motionStayProb[1];
        }
    }

	return prob;
}


/* Calculate observation probability */
double obsProb( bool obser, int position ) {
	double prob = 0;
	bool isDoor;
	/* QUESTION 4 HERE */

    if(position == 3 || position == 5 || position == 8)
    {
        isDoor = true;
    }
    else
    {
        isDoor = false;
    }

    if(obser)
    {
        if(isDoor)
        {
            prob = obsDoorProb[0];
        }
        else
        {
            prob = obsNotDoorProb[1];
        }
    }
    else
    {
        if(!isDoor)
        {
            prob = obsNotDoorProb[0];
        }
        else
        {
            prob = obsDoorProb[1];
        }

    }


	return prob;
}

/* Update our belief */
vector<double> bayesFilter( vector<double> preBelief, bool control, bool observation ) {
	vector<double> curBelief;
    vector<double> predict;
    predict.resize(preBelief.size());
	curBelief.resize( preBelief.size() );

    double sum = 0;
	/* QUESTION 5 HERE */
	/* Hint: First implement the prediction step (Equation (1) in the notes) */
    if(control)
    {
//        predict.at(0) = preBelief.back()* 0.7;
//        for (int i = 0; i < preBelief.size() - 1; i++)
//        {
//            predict.at(i + 1) = preBelief.at(i) * motionProb(i, i+1,control);
//        }

        predict.at(0) = preBelief.back()*0.7 + preBelief.at(preBelief.size() - 2)*0.3;
        predict.at(1) = preBelief.at(0)*0.7 + preBelief.back()*0.3;

        for (int i = 1; i < preBelief.size() - 1; i++)
        {
            predict.at(i+1) = predict.at(i-1)*0.3 + predict.at(i)*0.7;
        }

//        sum = vectorsum(predict);

//        for(int i = 0 ; i < predict.size(); i++)
//        {
//            predict.at(i) = predict.at(i)/sum;
//        }
    }
    else
    {
        for (int i = 0; i < preBelief.size() - 1; i++)
        {
            predict.at(i) = preBelief.at(i);
        }
    }


	/* 	 Then implement the update step (Equation (2) in the notes) */

    for (int i = 0; i < predict.size(); i++)
    {
        curBelief.at(i) = predict.at(i) * obsProb(observation,i+1);
    }

    double Constant = vectorsum(curBelief);  

    for(int i = 0; i < curBelief.size(); i++)
    {
        curBelief.at(i) = curBelief.at(i)/Constant;
    }


	return curBelief;

}






/*--------------------------MAIN FUNCTION---------------------------*/
int main(int argc, char *argv[]) {
	vector<double> initBelief;
	int num = 20;
    double probP = 1.0/num;
	initBelief.resize( num );
	// initialize the prior belief
	/* QUESTION 1 & 2 HERE */

    for (int i = 0; i < initBelief.size(); i++)
    {
        initBelief.at(i) = probP;
    }



	cout << "************  Q1 & Q2 **************************\n";
	cout << "The initialized probabilities are:\n";
	printvector( initBelief );
	cout << endl;





	/* Q3
	 * 		Calculate the probability given previous position,
	 * 		current position and control input
	 * 		Motion probability is given in motionMoveProb and
	 * 		motionStayProb */

	/* Testing your motionProb(...) function */
	int prePosition = 3, curPosition = 5;
	bool control = true;
	double mProb = motionProb( prePosition, curPosition, control );
	cout << "************  Q3 **************************\n";
	cout << "The motion probability is " << mProb << "\n";
	cout << endl;






	/* Q4
	 * 		Calculate the probability given observation and
	 * 		current position
	 * 		Observation probability is given in isDoorProb
	 * 		and noDoorProb */
	/* Testing your obsProb(...) function */
	bool observation = true;
    int position = 4;
	double oProb = obsProb( observation, position );
	cout << "************  Q4 **************************\n";
	cout << "The observation probability is " << oProb << "\n";
	cout << endl;




	/* Q5
	 * Testing your bayesFilter(...) function */
	vector<double> curBelief;
	curBelief = bayesFilter( initBelief, control, observation );
	cout << "************  Q5 **************************\n";
	cout << "Updated probabilities: \n";
	printvector( curBelief );
	cout << endl;




	/* Q6 HERE */
	/* Just call the bayesFilter(...) function 4 times according to the given
	   sequence of control inputs and observations */
	/* HINT: Look at Q5 Testing code */

    cout << "************  Q6 **************************\n";
    curBelief = bayesFilter(initBelief, false, true);
    cout << "(u0 = 0, z1 = 1) \n";
    printvector(curBelief);
    cout<<"SUM: "<<vectorsum(curBelief)<<endl;
    cout<<endl<<endl;

    curBelief = bayesFilter(curBelief, true, false);
    cout << "(u1 = 1, z2 = 0) \n";
    printvector(curBelief);
    cout<<"SUM: "<<vectorsum(curBelief)<<endl;
    cout<<endl<<endl;

    curBelief = bayesFilter(curBelief, true, true);
    cout << "(u2 = 1, z3 = 1) \n";
    printvector(curBelief);
    cout<<"SUM: "<<vectorsum(curBelief)<<endl;
    cout<<endl<<endl;

    curBelief = bayesFilter(curBelief, true, true);
    cout << "(u3 = 1, z4 = 1) \n";
    printvector(curBelief);
    cout<<"SUM: "<<vectorsum(curBelief)<<endl;
    cout<<endl<<endl;

    //cout<<"SUM: "<<vectorsum(curBelief)<<endl;



	return(0);
}
