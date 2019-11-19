function [StatementDot] = doDiff(Statement)

syms q0 q1 q2 q3 q4 real
syms q0dot q1dot q2dot q3dot q4dot real

StatementDot = diff(Statement,q0)*q0dot  + diff(Statement,q1)*q1dot + diff(Statement,q2)*q2dot + diff(Statement,q3)*q3dot+ diff(Statement,q4)*q4dot;


end