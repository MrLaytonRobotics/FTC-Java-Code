package org.firstinspires.ftc.teamcode.stellarstructure;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.stellarstructure.runnables.Runnable;
import org.firstinspires.ftc.teamcode.stellarstructure.conditions.Condition;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

public class Scheduler {
	private static final Scheduler instance = new Scheduler();

	private Scheduler() {}

	//todo: make the waitlist optional with a boolean
	//todo: add cancel method for runnables

	public static Scheduler getInstance() {
		return instance;
	}

	private final List<Subsystem> subsystems = new ArrayList<>();
	private final List<Runnable> runnableScheduleQueue = new ArrayList<>();
	private final List<Runnable> activeRunnables = new ArrayList<>();
	private final List<Trigger> activeTriggers = new ArrayList<>();

	private final List<Runnable> runnablesToAdd = new ArrayList<>();
	private final List<Runnable> runnablesToRemove = new ArrayList<>();

	public void addSubsystem(Subsystem subsystem) {
		subsystems.add(subsystem);
	}

	public void addTrigger(Trigger trigger) {
		activeTriggers.add(trigger);
	}

	public void removeTrigger(Trigger trigger) {
		activeTriggers.remove(trigger);
	}

	private boolean checkStartingConditions(@NonNull Runnable runnable) {
		Condition[] conditions = runnable.getStartingConditions();
		for (Condition condition : conditions) {
			if (!condition.evaluate()) {
				return false;
			}
		}

		return true;
	}

	private boolean startRunnable(Runnable runnableToStart) {
		boolean didInterrupt = false;

		// for every running directive
		for (Runnable activeRunnable : this.activeRunnables) {
			// check for conflicts
			CONFLICT_CHECK:
			// for every subsystem required by the new directive
			for (Subsystem requiredByNew : runnableToStart.getRequiredSubsystems()) {
				// for every subsystem required by the running directive
				for (Subsystem requiredByRunning : activeRunnable.getRequiredSubsystems()) {
					if (requiredByNew == requiredByRunning) {
						// CONFLICT FOUND!

						if (activeRunnable.getInterruptible()) {
							// if the running command is interruptible, stop it and remove it
							activeRunnable.schedulerStop(true);

							// remove the runnable's triggers
							activeTriggers.removeAll(activeRunnable.getOwnedTriggers());

							runnablesToRemove.add(activeRunnable);
							didInterrupt = true;
						} else {
							// running command unable to be interrupted, so can't schedule new directive
							return false;
						}

						// checked requirements for this runningDirective, move to next
						break CONFLICT_CHECK;
					}
				}
			}
		}

		this.activeRunnables.add(runnableToStart); // add to running directives

		//add runnable's triggers
		for (Trigger trigger : runnableToStart.getOwnedTriggers()) {
			addTrigger(trigger);
		}

		runnableToStart.schedulerStart(didInterrupt); // start directive and pass hadToInterruptToStart status
		return true;
	}

	public void schedule(@NonNull Runnable runnableToSchedule) {
		// prevent scheduling of the same directive multiple times
		if (this.runnableScheduleQueue.contains(runnableToSchedule) || this.activeRunnables.contains(runnableToSchedule)) {
			return;
		}

		// check for starting conditions
		if (!checkStartingConditions(runnableToSchedule)) {
			this.runnablesToAdd.add(runnableToSchedule);
			return;
		}

		// try to run
		if (!startRunnable(runnableToSchedule)) {
			// didn't start, so add to queue
			this.runnableScheduleQueue.add(runnableToSchedule);
		}
	}

	public void checkScheduleQueue() {
		for (Iterator<Runnable> iterator = this.runnableScheduleQueue.iterator(); iterator.hasNext(); ) {
			Runnable runnable = iterator.next();
			if (checkStartingConditions(runnable)) {
				if (startRunnable(runnable)) {
					iterator.remove();
				}
			}
		}
	}

	public void run() {
		// check schedule queue and potentially move directives to running directives
		checkScheduleQueue();

		//todo: make the queuing system work

		// stop directives
		for (Iterator<Runnable> iterator = this.runnablesToAdd.iterator(); iterator.hasNext();) {
			Runnable runnable = iterator.next();
			this.activeRunnables.remove(runnable);
			runnable.schedulerStop(true);
		}

		// start directives
		for (Runnable runnable : this.runnablesToAdd) {
			this.activeRunnables.add(runnable);
			runnable.schedulerStart(false);
		}

		// check and run all triggers
		for (Trigger trigger : this.activeTriggers) {
			if (trigger.check()) {
				trigger.run();
			}
		}

		// update directives and remove finished directives
		for (Runnable runnable : this.activeRunnables) {
			if (runnable.getFinished()) {
				// remove the runnable's triggers
				activeTriggers.removeAll(runnable.getOwnedTriggers());

				runnablesToRemove.add(runnable);
			} else {
				runnable.update();
			}
		}

		// if subsystem isn't being used, then schedule default directive
		for (Subsystem subsystem : this.subsystems) {
			Runnable defaultDirective = subsystem.getDefaultDirective();

			if (defaultDirective != null && !isSubsystemInUse(subsystem)) {
				schedule(defaultDirective);
			}
		}
	}

	private boolean isSubsystemInUse(Subsystem subsystemToCheck) {
		// for every running directive
		for (Runnable activeRunnable : this.activeRunnables) {
			// check if running directive requires subsystem
			// includes the default directive itself but that's fine for this application
			for (Subsystem requiredSubsystem : activeRunnable.getRequiredSubsystems()) {
				if (requiredSubsystem == subsystemToCheck) {
					return true;
				}
			}
		}

		return false;
	}

	public void cancelAll() {
		// clear all queued directives
		this.runnableScheduleQueue.clear();

		// remove all triggers
		this.activeTriggers.clear();

		// stop all directives
		for (Runnable runnable : this.activeRunnables) {
			runnable.schedulerStop(true);
		}

		// clear all running directives
		this.activeRunnables.clear();
	}

	public String getTelemetry() {
		return String.format("Runnables Queue: %d\nActive Runnables: %d", this.runnableScheduleQueue.size(), this.activeRunnables.size());
	}
}
