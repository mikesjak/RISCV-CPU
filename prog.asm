.data
.align 2

array:
.word 1 3 5 7 9

.text # my code is below
j main
    
isPrime:
  addi t0, zero, 0x2     # Initial value for Index
  add t1, zero, a1       # Max value
  addi t2, zero, 0       # rem value
  addi a0, zero, 1       # return value
  #add t6, zero, ra       # return adress
  beq a1, a0, notPrime   # if Num = 1 -> notPrime
  beq a1, zero, notPrime # if Num = 0 -> notPrime
  
  forac:
    beq t0, t1, done     # if index = maxVal, done
    rem t2, a1, t0       # t2 = a1%t0
    beq t2, zero, notPrime # if t2 == 0, notPrime
  plus:
    addi t0, t0, 1 # i++
    j forac 
  notPrime:
    addi a0, zero, 0 # return 0
  done:
    #add ra, zero, t6
    ret
    
main:
   lw s0, 0x00000008
   lw s2, 0x00000004

  addi s1, zero, 0    # Cycle index
  j for
  
noPrime:
  sw zero, 0x0(s0) # write 0 to address s0
  j skip  

prime:
  sw a0, 0x0(s0)  # write 1 to address s0
  j skip
  
for:
  beq s1, s2, end # if index = maxVal
  lw a1, 0x0(s0)  # load current Num
  jal isPrime     # call isPrime
  beq a0, zero, noPrime # if a0 == 0, noPrime
  j prime             # else: prime
  skip:
    addi s0, s0, 0x4    # address += 4
    addi s1, s1, 0x1    # index++;
    j for
  
end:
nop
  j end
  
  
