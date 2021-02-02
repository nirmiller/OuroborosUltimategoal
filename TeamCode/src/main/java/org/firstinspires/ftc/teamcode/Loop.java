package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class Loop {

    ArrayList<Thread> threads;

    public Loop(){

        threads = new ArrayList<>();

    }

    public void add(Thread thread){
        threads.add(thread);
    }

    public void add(ArrayList<Thread> threads2){
        for(Thread t : threads2){
            threads.add(t);
        }
    }

    public void run(){


        if(threads.size() < 1){
            return;
        }
        for(Thread t : threads){
            t.start();
        }

        while(threads.get(threads.size() - 1).isAlive()){

        }
        end();


    }
    public void clear(){
        threads.clear();
    }
    public void end(){
        if(threads.size() < 1){
            return;
        }
        for(Thread t : threads){
            t.interrupt();
        }
    }

}
