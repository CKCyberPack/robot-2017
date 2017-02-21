package org.usfirst.frc.team5689;

import static org.usfirst.frc.team5689.DriveRunnable.Status.CANCELLED;
import static org.usfirst.frc.team5689.DriveRunnable.Status.PENDING;

public abstract class DriveRunnable implements Runnable {

    private Status status = PENDING;
    private boolean cancelled = false;

    public Status getStatus() {
        return status;
    }

    protected final void setStatus(Status newStatus) {
        status = newStatus;
    }

    protected boolean isCancelled() {
        return cancelled;
    }

    public void start() {
        new Thread(this).start();
    }

    public final void cancel() {
        cancelled = true;
        setStatus(CANCELLED);
    }

    public void runSync() {
        this.run();
    }

    enum Status {
        PENDING,
        RUNNING,
        FINISHED,
        DEAD,
        CANCELLED
    }

}
