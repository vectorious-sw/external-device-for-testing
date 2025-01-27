{
where = match($1, "Revision")
if (where != 0)
	print ("#define PCCOMMAPPLAYER_FW_VERSION  " $2) 
}
