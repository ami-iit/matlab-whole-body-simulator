function  [qp_output, stop] = ProcessQPOutputFCN(qpStatus, qpSolution)

persistent oldQPSolution;
persistent counter;

coder.extrinsic('warning')
if isempty(oldQPSolution)
    oldQPSolution = zeros(size(qpSolution));
end

if isempty(counter)
    counter = 0.0 ;
end

if(qpStatus == 0.0)
    qp_output = qpSolution;
    oldQPSolution = qpSolution;
    counter = 0.0;
    stop = 0.0;
    
else
    counter = counter +1.0;
    qp_output = oldQPSolution;
    stop = 0.0;
    
    warning("QPOasesError: Sending last feaseable solution")
    if (counter>2.0)
        stop = 1.0;
        warning("QPOasesError: Stopping the Controller")
    end
end
