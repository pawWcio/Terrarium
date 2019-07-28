////SEND
//
//String frame;                              //ciag wysylanych bitow
//String crc;                                //kod crc
//String encoded="";                            //zakodowane bity
//int frame_size= sizeof(frame);               //ilosc wysylanych bitow
//int crc_size= sizeof(crc);                   //dlugosc kodu crc
//int encoded_size= sizeof(encoded);           //dlugosc zdekodowanych bitow
//
//encoded += frame;
//for(int i=1; i<=crc_size-1; i++){             //dodajemy wyzerowane bity
//  encoded+='0';   
//}
//
//for(int i=0; i<= encoded_size-crc_size; ){
//  for(int j=0; j< crc_size; j++){
//    encoded[i+j]=encoded[i+j]==crc[j]? '0':'1';    //XOR
//  }
//  for( ; i<encoded_size && encoded[i]!='1'; i++ );   //przesuwamy dzielnik az do napotkania jedynki
//}
//
//String encoded_crc = encoded.substring((encoded_size-_crc_size), encoded_size+1);   //wyodrebniony kod crc
//String encoded_frame = frame+encoded_crc;                                         //wyodrebniona ramka z dodanym kodem crc
//
//
////RECEIVE
//
//for(int i=0; i<= encoded_size-crc_size; ){
//  for(int j=0; j< crc_size; j++){
//    encoded[i+j]=encoded[i+j]==crc[j]? '0':'1';    //XOR
//  }
//  for( ; i<encoded_size && encoded[i]!='1'; i++ );
//}
//for(char i: encoded.substring(encoded_size-crc_size)) {
//    if (i!='0') //error in cmmunication
//    else        //communication correct
//}
