all: bayesFilter

bayesFilter: bayesFilter.o
	g++ bayesFilter.o -o bayesFilter

bayesFilter.o: bayesFilter.cpp
	g++ -c  bayesFilter.cpp


clean:
	rm -rf *.o bayesFilter
