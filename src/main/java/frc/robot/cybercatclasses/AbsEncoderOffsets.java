package frc.robot.cybercatclasses;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

public class AbsEncoderOffsets {
    public double frontLeft;
    public double frontRight;

    public AbsEncoderOffsets(){}
    
    public AbsEncoderOffsets(double fl, double fr){
        this.frontLeft = fl;
        this.frontRight = fr;
    }

    public static String getJsonEncoding(AbsEncoderOffsets absEncoderOffsets){
        ObjectMapper objectMapper = new ObjectMapper();
        String jsonEncoding = "";
        try{
            return objectMapper.writeValueAsString(absEncoderOffsets);
        } catch(JsonProcessingException e){
            e.printStackTrace();
        }
        return jsonEncoding;
    }
}
