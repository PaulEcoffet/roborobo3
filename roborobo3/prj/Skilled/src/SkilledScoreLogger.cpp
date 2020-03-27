//
// Created by pecoffet on 14/11/2019.
//

#include <core/RoboroboMain/main.h>
#include "Skilled/include/SkilledScoreLogger.h"

void SkilledScoreLogger::addScore(int id, int cost, int nbpart, double owncoop, double othercoop, double score) {
    file << m_eval << "," << m_iter << "," << id << "," << cost << "," << nbpart << "," << owncoop << ","
         << othercoop << "," << score << "\n";
}

void SkilledScoreLogger::close() {
    file.flush();
    file.close();
    std::cout << "log " << m_actupath << " bien fermé" << std::endl;
}

void SkilledScoreLogger::updateEval(int eval) {
    m_eval = eval;
}

void SkilledScoreLogger::updateIter(int iter) {
    m_iter = iter;
}

void SkilledScoreLogger::openNewLog(int gen) {
    file.close();
    std::stringstream path;
    path << gLogDirectoryname;
    path << "/logscore_" << gen << ".txt.gz";
    const std::string pathstr = path.str();
    m_actupath = pathstr;
    file.open(pathstr.c_str());
    file << "eval,iter,id,cost,nbpart,owncoop,othercoop,score\n";
    std::cout << pathstr << " open." << std::endl;
}