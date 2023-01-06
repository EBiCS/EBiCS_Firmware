import java.io.*;
import java.util.Scanner;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
public class hexToLsh {
  public static void main(String[] args) {
    try {
      String hexFile = "build/EBiCS_Firmware";

      File myObj = new File(hexFile + ".hex");
      Scanner myReader = new Scanner(myObj);

      int maxAddress = 0;
      while (myReader.hasNextLine()) {
        String data = myReader.nextLine();
        if (data.substring(7, 9).equals("00") && Integer.parseInt(data.substring(3, 7), 16) > maxAddress) {
          maxAddress = Integer.parseInt(data.substring(3, 7), 16);
        }
      }
      myReader.close();

      int limitAddress = 16 * ((maxAddress / 16) + 2);
      System.out.println("max address is: " + Integer.toHexString(maxAddress) + ", will fill up to: "
          + Integer.toHexString(limitAddress));
      String limitAddressStr = "0x08" + String.format("%6s", Integer.toHexString(limitAddress)).replace(' ', '0');

      Process process = new ProcessBuilder("srec_cat", hexFile + ".hex", "-Intel", "-fill", "0x00", "0x08001000", limitAddressStr,
          "-o", hexFile + ".filled", "-Intel", "-Output_Block_Size=16").start();

      process.waitFor();

      File filledHex = new File(hexFile + ".filled");
      Scanner filledHexReader = new Scanner(filledHex);

      int[] key = { 0x81, 0x30, 0x00, 0x5a, 0x7f, 0xcb, 0x37, 0x13, 0x32, 0x85, 0x20, 0x4b, 0xc8, 0xf3, 0x10, 0x2e,
          0x1c, 0xa7, 0xc2, 0xa3 };
      
      String folderName = "output";
      Path path = Paths.get(folderName);
      Files.createDirectory(path);
      File from = new File("Inc/config.h");
      File to = new File("output/config.h");
      Files.copy(from.toPath(), to.toPath());
      from = new File("Inc/main.h");
      to = new File("output/main.h");
      Files.copy(from.toPath(), to.toPath());

      String lshFile = "output/EBiCS_Firmware";
      FileWriter myWriter = new FileWriter(lshFile + ".lsh");

      int lineIndex = 0;
      while (filledHexReader.hasNextLine()) {
        String dataF = filledHexReader.nextLine();

        String input = dataF.substring(3);

        int index = 0;
        StringBuilder output = new StringBuilder();

        if (input.substring(4, 6).equals("00")) {
          for (String hex : input.replaceAll("..(?!$)", "$0,").split(",")) {
            output.append(
                String.format("%2s", Integer.toHexString(Integer.parseInt(hex, 16) ^ key[index++])).replace(' ', '0'));
          }

          String newline = lineIndex == 0 ? "" : "\n";
          myWriter.write(newline + output.toString().toUpperCase());

          lineIndex++;
        }
      }
      filledHexReader.close();
      myWriter.close();
    } catch (FileNotFoundException e) {
      System.out.println("An error occurred.");
      e.printStackTrace();
    } catch (IOException e) {
      System.out.println("An error occurred.");
      e.printStackTrace();
    } catch (InterruptedException e) {
      System.out.println("An error occurred.");
      e.printStackTrace();
    }
  }
}
