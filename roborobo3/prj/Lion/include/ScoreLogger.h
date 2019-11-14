//
// Created by pecoffet on 14/11/2019.
//

#ifndef ROBOROBO3_SCORELOGGER_H
#define ROBOROBO3_SCORELOGGER_H


#include <gzstream.h>

class ScoreLogger
{
public:
    void addScore(int id, int cost, int nbpart, double owncoop, double othercoop, double score);

    void close();

    void updateEval(int eval);

    void updateIter(int iter);

    void openNewLog(int gen);

private:
    ogzstream file;
    int m_eval;
    int m_iter;

};


#endif //ROBOROBO3_SCORELOGGER_H
