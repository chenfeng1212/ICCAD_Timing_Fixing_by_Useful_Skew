#ifndef SPFA_H
#define SPFA_H

#include "ds.h"
#include <queue>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace skew {
    namespace SPFA {
        inline void spfa(DesignDB& db){
            //Timer timer("SPFA");
            //std::cout << "into SPFA" << std::endl;
            int ffsize=db.ffs.size();

            std::vector<int> times;
            std::vector<bool> inQueue;
            std::queue<int> q;
            bool stop;

            for (auto& FF : db.ffs) {
                FF.targetArrivalFF=FF.clkFF;
                FF.targetArrivalSS=FF.clkSS;
            }

            db.buildConstraintGraph(ConstraintKind::SETUP);
            times.assign(ffsize,0);
            inQueue.assign(ffsize, false);
            q =std::queue<int>();
            stop=false;

            for(int i=0;i<db.ffs.size();i++){
                q.push(i);
                times[i]=1;
                inQueue[i]=true;
            }

            while(!q.empty()&&!stop){//setup SS
                int launch = q.front();
                q.pop();

                inQueue[launch] = false;

                for(auto &con : db.spfaAdj[launch]){
                    int capture = con.to;

                    if(db.ffs[launch].targetArrivalSS - db.ffs[capture].targetArrivalSS > con.weight){
                        db.ffs[capture].targetArrivalSS = db.ffs[launch].targetArrivalSS-con.weight;
                        if(!inQueue[capture]){
                            q.push(capture);
                            inQueue[capture] = true;
                            times[capture]++;
                            if(times[capture]>=ffsize)stop=true;
                        }
                    }
                }
            }


            db.buildConstraintGraph(ConstraintKind::HOLD);
            times.assign(ffsize,0);
            inQueue.assign(ffsize, false);
            q =std::queue<int>();
            stop=false;

            for(int i=0;i<db.ffs.size();i++){
                q.push(i);
                times[i]=1;
                inQueue[i]=true;
            }

            while(!q.empty()&&!stop){//hold FF
                int capture = q.front();
                q.pop();

                inQueue[capture] = false;

                for(auto &con : db.spfaAdj[capture]){
                    int launch = con.to;

                    if(db.ffs[capture].targetArrivalFF - db.ffs[launch].targetArrivalFF > con.weight){
                        db.ffs[launch].targetArrivalFF = db.ffs[capture].targetArrivalFF - con.weight;

                        if(!inQueue[launch]){
                            q.push(launch);
                            inQueue[launch] = true;
                            times[launch]++;
                            if(times[launch]>=ffsize)stop=true;
                        }
                    }
                }
            }
            
            // add calculate target shift
            for(auto& FF : db.ffs){
                FF.targetShiftSS =
                    FF.targetArrivalSS - FF.clkSS;

                FF.targetShiftFF =
                    FF.targetArrivalFF - FF.clkFF;                
            }
        }
    }
}






#endif