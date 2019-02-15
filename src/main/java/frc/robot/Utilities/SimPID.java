package frc.robot.Utilities;

public class SimPID {
    
	private double pConst;
	private double iConst;
	private double dConst;
	private double desiredVal;
	private double previousVal;
	private double errorSum;
	private double errorIncrement;
	private double errorEpsilon;
	private double doneRange;
	private boolean firstCycle;
	private double maxOutput;
	private int minCycleCount;
	private int cycleCount;
	
	public SimPID() {
		this(0.0, 0.0, 0.0, 0.0);
	}
	
	public SimPID(double p, double i, double d, double eps) {
		this.pConst = p;
		this.iConst = i;
		this.dConst = d;
		this.errorEpsilon = eps;
		this.doneRange = eps;
		
		this.desiredVal = 0.0;
		this.firstCycle = true;
		this.maxOutput = 1.0;
		this.errorIncrement = 1.0;
		
		this.cycleCount = 0;
		this.minCycleCount = 5;
	}
	
	public SimPID(double p, double i, double d) {
		this(p, i, d, 1.0);
	}
	
	public void setConstants(double p, double i, double d) {
		this.pConst = p;
		this.iConst = i;
		this.dConst = d;
	}
	
	public void setDoneRange(double range) {
		this.doneRange = range;
	}
	
	public void setErrorEpsilon(double eps) {
		this.errorEpsilon = eps;
	}
	
	public void setDesiredValue(double val) {
		this.desiredVal = val;
	}
	
	public void setMaxOutput(double max) {
		if(max < 0.0) {
			this.maxOutput = 0.0;
		} else if(max > 1.0) {
			this.maxOutput = 1.0;
		} else {
			this.maxOutput = max;
		}
	}
	
	public void setMinDoneCycles(int num) {
		this.minCycleCount = num;
	}
	
	public void resetErrorSum() {
		this.errorSum = 0.0;
	}
	
	public double getDesiredVal() {
		return this.desiredVal;
	}
	
	public double getPreviousVal() {
		return this.previousVal;
	}
	
	public double calcPID(double currentVal) {
		
		double pVal = 0.0;
		double iVal = 0.0;
		double dVal = 0.0;
		
		if(this.firstCycle) {
			this.previousVal = currentVal;
			this.firstCycle = false;
		}
		
		///////P Calc///////
		double error = this.desiredVal - currentVal;
		pVal = this.pConst * error;
		
		///////I Calc///////
		
		//+ error outside of acceptable range
		if(error > this.errorEpsilon) {
			//check if error sum was in the wrong direction
			if(this.errorSum < 0.0) {
				this.errorSum = 0.0;
			}
			//only allow up to the max contribution per cycle
			this.errorSum += Math.min(error, this.errorIncrement);
		} //- error outside of acceptable range
		else if(error < -1.0 * this.errorEpsilon) {
			//error sum was in the wrong direction
			if(this.errorSum > 0.0) {
				this.errorSum = 0.0;
			}
			//add either the full error or the max allowable amount to sum
			this.errorSum += Math.max(error, -1.0 * this.errorIncrement);
		}
		//within the allowable epsilon
		else {
			//reset the error sum
			this.errorSum = 0.0;
		}
		//i contribution (final) calculation
		iVal = this.iConst * this.errorSum;
		
		///////D Calc///////
		double deriv = currentVal - this.previousVal;
		dVal = this.dConst * deriv;
		
		//overal PID calc
		double output = pVal + iVal - dVal;
		
		//limit the output
		output = limitValue(output, this.maxOutput);
		
		//store current value as previous for next cycle
		this.previousVal = currentVal;
		
		return output;
	}
	
	public boolean isDone() {
		double currError = Math.abs(this.desiredVal - this.previousVal);
		
		//close enough to target
		if(currError <= this.doneRange) {
			this.cycleCount++;
		}
		//not close enough to target
		else {
			this.cycleCount = 0;
		}
		
		return this.cycleCount > this.minCycleCount;
	}
	
	public void resetPreviousVal() {
		this.firstCycle = true;
	} 
	
	public double getMaxOutput() {
		return this.maxOutput;
	}

	public static double limitValue(double val) {
        return limitValue(val, 1.0);
    }
    
    public static double limitValue(double val, double max) {
        if(val > max) {
            return max;
        } else if(val < -max) {
            return -max;
        } else {
            return val;
        }
    }
}
