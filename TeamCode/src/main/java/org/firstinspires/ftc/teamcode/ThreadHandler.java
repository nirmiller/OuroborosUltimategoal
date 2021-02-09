package org.firstinspires.ftc.teamcode;

public class ThreadHandler {
    Thread thread;


    public ThreadHandler(){

        thread = null;
    }

    public void queue(Thread new_thread){

        if(thread == null){
            this.thread = new_thread;
        }else{
            if(this.thread.isAlive())
            {
                this.thread.interrupt();
            }
            this.thread = new_thread;
        }

        this.thread.start();
    }

    public void th_kill(){

        if(this.thread != null){
            if(live()){
                this.thread.interrupt();
            }
        }

    }

    public boolean live(){
        if(this.thread != null){
            return this.thread.isAlive();
        }
        return false;
    }

}
