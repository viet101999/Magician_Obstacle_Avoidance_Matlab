function p = findClosestPointOnLine(lineStart, lineEnd, pt)
        %calculate the point on a line that is closest to another point
        
        %calculate the line vector
        lineSeg = lineStart-lineEnd;
        %calculate the vector from the point to the end of the line
        ptToLine = pt-lineEnd;
        
        %calculate the dot products of the line to the point vector
        %and the line to itself
        c1 = dot(ptToLine,lineSeg,2);
        c2 = dot(lineSeg,lineSeg,2);
        %if c1 <= 0, the point is closer to the end of the line segment
        if c1 <= 0
            p = lineEnd;
            return;
            %if c2 <= c1, the point is closer to the start of the line segment
        elseif c2 <= c1
            p = lineStart;
            return;
        end
        %otherwise, calculate the point along the line that is closest
        %to the point
        lambda = c1/c2;
        p = lineEnd + lambda*lineSeg;
    end