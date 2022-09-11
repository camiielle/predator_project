#include "TAxis.h"
#include "TCanvas.h"
#include "TF1.h"
#include "TH1C.h"
#include "TStyle.h"
#include <iostream>

void macro()
{
  //
  // Canvas dove il grafico verra' disegnato
  // (dimensione : 500 x 500) ed opzioni di stile (griglie, colore di sfondo,
  // ...)
  //

  std::string const filename{"pred10_seek2"};

  TCanvas* canvas =
      new TCanvas("canvas", "Istogramma prede catturate", 1400, 1000);
  gStyle->SetCanvasColor(0);
  gStyle->SetStatBorderSize(1);
  canvas->SetGrid(true);

  //
  // Istogramma nBins = numero di bins , xMin(Max) = minimo (massimo) per l'asse
  // x riempito col numero di occorrenze per bin normalizzate a numero_di_misure
  // * largezza_del_bin
  //

  std::ifstream file{"data2/" + filename};
  std::string null;
  std::getline(file, null);

  TH1C* histo = new TH1C("histo", "Prede Catturate", 120, 1, 120);
  // histo->SetLabelFont(132);

  for (int c; file >> c; histo->Fill(c))
    ;

  histo->GetXaxis()->SetTitle("Numero di prede catturate");
  histo->GetYaxis()->SetTitle("Occorrenze");
  histo->SetFillColor(kAzure + 2);

  histo->Draw("HIST");

  canvas->Print((filename + ".pdf").c_str());
}
