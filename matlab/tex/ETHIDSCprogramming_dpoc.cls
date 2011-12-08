%==============================================================================
% ETHIMRTlec_dpoc
% This class is based on ETHIMRTlec with some modifications specific to lecture "Dynamic Programming and Optimal Control" in Fall 2008.
%
% ETHIMRTlec -- (ETHIMRTlec.cls) Class file
% A LaTeX class providing a header (ETH-IMRT) and some new commands
% Author: Eric A. Mueller
%         Measurement and Control Laboratory
%         Swiss Federal Institute of Technology (ETH), Zurich
%         Zurich, Switzerland
%
% Copyright (c) 2007 Eric A. Mueller
% All rights reserved.
% 
% Permission is hereby granted, without written agreement and without
% license or royalty fees, to use, copy, modify, and distribute this
% software and its documentation for any purpose, provided that the
% above copyright notice and the following two paragraphs appear in
% all copies of this software.
% 
% IN NO EVENT SHALL THE AUTHOR BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, 
% SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF 
% THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE AUTHOR HAS BEEN ADVISED 
% OF THE POSSIBILITY OF SUCH DAMAGE.
% 
% THE AUTHOR SPECIFICALLY DISCLAIMS ANY WARRANTIES,
% INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
% AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
% ON AN "AS IS" BASIS, AND THE AUTHOR HAS NO OBLIGATION TO
% PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
%
% ETHIMRTlec.cls,v 1.4.3 2007/10/16
%
% Revision history:
% 2008/09/23, Sebastian Trimpe (ST)
%							created class ETHIMRTlec_dpoc based on ETHIMRTlec
% 2008/09/23, ST		* swapped prof names; now R. D'Andrea, L. Guzzella
% 							* added option 'sheet' for blank sheet (e.g. for class 
%							information)
%							* fontsize set to 10
%							* adapted option 'uebung'
% 2009/10/18m Sebastian Trimpe
%							* modifications to be used for programming exercises
%							in class Dyn Prog & Opt Control in Fall 2009
%==============================================================================
\def\Presentation@Version{1.4.1}
\NeedsTeXFormat{LaTeX2e}[1995/12/01]
\ProvidesClass{ETHIDSCprogramming_dpoc}[2008/09/23, v. \Presentation@Version]
\typeout{Providing ETH-IMRT header and some new commands ---}
\typeout{Eric A. Mueller, ETHZ, Switzerland}
\typeout{ }


%%%%%%%
\newif\ifuebung
\DeclareOption{uebung}{\global\uebungtrue}
\newif\ifloesung
\DeclareOption{loesung}{\global\loesungfalse}
\newif\ifsheet
\DeclareOption{sheet}{\global\sheettrue}
\newif\ifnoname
\DeclareOption{noname}{\global\nonametrue}
\newif\ifnodate
\DeclareOption{nodate}{\global\nodatetrue}
\newif\ifnovisum
\DeclareOption{novisum}{\global\novisumtrue}



\ExecuteOptions{uebung}
\ProcessOptions

%%%%%%%
\LoadClass[11pt,twoside,fleqn,a4paper]{article}
\RequirePackage{vmargin}
\RequirePackage{graphicx}
\RequirePackage[fleqn]{amsmath}
\RequirePackage{caption}
\RequirePackage{a4}
\RequirePackage[german]{babel}
\RequirePackage{amsfonts}
\RequirePackage{amssymb}
\RequirePackage[latin1]{inputenc}
\RequirePackage{float}
\RequirePackage{srcltx}

%%%%%%%
\ifuebung
\pagestyle{empty}
\setmarginsrb{26mm}{25mm}{24mm}{20mm}{0mm}{0mm}{0mm}{0mm}
% {leftmargin}{topmargin}{rightmargin}{bottommargin}
%     {headheight}{headsep}{footheight}{footskip}
\else
\setmarginsrb{26mm}{25mm}{24mm}{12.5mm}{0mm}{0mm}{10mm}{12.5mm}
% {leftmargin}{topmargin}{rightmargin}{bottommargin}
%     {headheight}{headsep}{footheight}{footskip}
\fi

\setlength{\parindent}{0pt}

% Caption font
\renewcommand{\captionfont}{\slshape}

%%%%%%%
\newcommand{\zweis}{
\vfill\hfill{\tiny $\cdot/\cdot$}\pagebreak}

\newcommand{\@vorlesungstitel}{}
\newcommand{\vorlesungstitel}[1]{\gdef\@vorlesungstitel{#1}}

\newcommand{\@vorlesungsnummer}{}
\newcommand{\vorlesungsnummer}[1]{\gdef\@vorlesungsnummer{#1}}

\newcommand{\@semester}{}
\newcommand{\semester}[1]{\gdef\@semester{#1}}

\newcommand{\@uebungsnummer}{}
\newcommand{\uebungsnummer}[1]{\gdef\@uebungsnummer{#1}}

\newcommand{\@thema}{}
\newcommand{\thema}[1]{\gdef\@thema{#1}}

\newcommand{\@ausgabe}{}
\newcommand{\ausgabe}[1]{\gdef\@ausgabe{Issued:\, #1}}

\newcommand{\@vorbesprechung}{}
\newcommand{\vorbesprechung}[1]{\gdef\@vorbesprechung{Vorbesprechung:\, #1}}

\newcommand{\@abgabe}{}
\newcommand{\abgabe}[1]{\gdef\@abgabe{Due:\, #1}}

\newcommand{\@nachbesprechung}{}
\newcommand{\nachbesprechung}[1]{\gdef\@nachbesprechung{Recitation:\, #1}}

\newcommand{\@info}{}
\newcommand{\info}[1]{\gdef\@info{#1}}

\newcounter{aufg}
\newcommand{\aufg}[1]{\vspace{0.8ex plus0.2ex minus0.2ex}
 \stepcounter{aufg}
 \subsection*{Problem \arabic{aufg} \ (#1)}}

\newcommand{\@textbook}{}
\newcommand{\textbook}[1]{\gdef\@textbook{#1}}
\newcommand{\bookexercise}[2]{\@textbook, p.~#2, exercise~#1}


%%%%%
% enumeration of exercises
\def\exercises{\enumerate}
\def\endexercises{\endenumerate}
\def\newexercise{\par\item}


%%%%%%%
\newcounter{teilaufg}
\newenvironment{taufg}{
 \begin{list}{\alph{teilaufg})\hfill}{\usecounter{teilaufg}
 \setlength{\labelsep}{1.25em}
 \setlength{\labelwidth}{1.25em}
 \setlength{\leftmargin}{2.5em}
 \setlength{\itemindent}{0cm}}}{
 \end{list}}

\newcounter{tteilaufg}
\newenvironment{ttaufg}{
 \begin{list}{{\roman{tteilaufg})}\hfill}{\usecounter{tteilaufg}
 \setlength{\labelsep}{0.75em}
 \setlength{\labelwidth}{1.75em}
 \setlength{\leftmargin}{2.5em}
 \setlength{\itemindent}{0cm}}}{
 \end{list}}

\newcounter{enumL}
\newenvironment{enumL}{
 \begin{list}{{\arabic{enumL}.}\hfill}{\usecounter{enumL}
 \setlength{\labelsep}{0.75em}
 \setlength{\labelwidth}{1.75em}
 \setlength{\leftmargin}{2.5em}
 \setlength{\itemindent}{0cm}}}{
 \end{list}}
 
\newenvironment{aufz}{
 \begin{list}{$\bullet$}{
 \setlength{\labelsep}{1.25em}
 \setlength{\labelwidth}{1.25em}
 \setlength{\leftmargin}{2.5em}
 \setlength{\itemindent}{0cm}
 \setlength{\itemsep}{0cm}}}{
 \end{list}} 
 
\newenvironment{aufzL}{
 \begin{list}{$\bullet$\hfill}{
 \setlength{\labelsep}{1.25em}
 \setlength{\labelwidth}{1.25em}
 \setlength{\leftmargin}{2.5em}
 \setlength{\itemindent}{0cm}
 \setlength{\itemsep}{0cm}}}{
 \end{list}} 

%%%%%%%
\ifuebung
\renewcommand{\maketitle}{
    \pagestyle{empty}
    {\fontsize{10}{15}\fontfamily{cmss}\selectfont
    \vspace*{-1.3cm}
    \begin{minipage}[b]{\textwidth}
    \begin{minipage}[t]{0.4\textwidth}
    \hspace{-0.33cm}
    \includegraphics[scale=0.8]{img/ethlogo}
    \end{minipage}
    \hfill
    \begin{minipage}[t]{0.6\textwidth}
    \vspace*{-1.3cm}
    \begin{flushright}
    \vspace{-0.3cm}
    \includegraphics[height=1.4cm]{img/IDSC_logo_Full}\\
    \vspace{0.40cm}
    %{\fontsize{6}{8}\fontshape{sl}\selectfont
    %Prof. R. D'Andrea\hspace*{2.2pt}\par}
    \end{flushright}
    \end{minipage}
    \end{minipage}
    \begin{minipage}{\textwidth}
    \vspace{0.7ex}
    \begin{minipage}[t]{0.14\textwidth}
    \@vorlesungsnummer
    \end{minipage}
    \hfill
    \begin{minipage}[t]{0.86\textwidth}
    \textbf{\Large \@vorlesungstitel} \ \if\@semester\else(\@semester)\fi
    \end{minipage}\\[0.3cm]
    \rule[0.1cm]{\textwidth}{.5pt}
	 \textbf{\large Programming Exercise \#\@uebungsnummer\hspace{1pt}}
	 \hfill
    \large Topic:  \@thema
    \\[0.25cm]\normalsize
    %\begin{minipage}[t]{0.14\textwidth}
    %\textbf{Topic:} 
    %\end{minipage}
    %\hfill 
    %\begin{minipage}[t]{0.86\textwidth}
    %\textbf{\@thema} 
    %\end{minipage}\\[0.1cm]
    \ifnodate
    \ifnoname
    \vskip-.9cm
    \fi
    \else
    \if.\@ausgabe\else\@ausgabe\hfill\fi
    %\if.\@vorbesprechung\else\@vorbesprechung\hspace{3.5em minus4em}\fi 
    \if.\@abgabe\else\@abgabe\hfill\fi
    \if.\@nachbesprechung\else\@nachbesprechung\fi 
	 \\
    \rule{\textwidth}{.8pt}
    \vspace{-1.3cm}
    \begin{flushright}
    {\tiny \if\@info\else{\vspace{0.3cm}\@info\vspace{0.1cm}}\fi}
    \end{flushright}
    \end{minipage}}}
\fi 

\ifloesung
\renewcommand{\maketitle}{
    {\fontsize{10}{15}\fontfamily{cmss}\selectfont
    \vspace*{-1.3cm}
    \begin{minipage}[b]{\textwidth}
    \begin{minipage}[t]{0.4\textwidth}
    \hspace{-0.33cm}
    \includegraphics[scale=0.8]{img/ethlogo}
    \end{minipage}
    \hfill
    \begin{minipage}[t]{0.6\textwidth}
    \vspace*{-1.3cm}
    \begin{flushright}
    \vspace{-0.3cm}
    \includegraphics[scale=1.2]{img/mcllogo}\\
    \vspace{-0.1cm}
    {\fontsize{6}{8}\fontshape{sl}\selectfont
    Prof. R. D'Andrea\hspace*{2.2pt}\linebreak 
    Prof. L. Guzzella\hspace*{2.2pt}\par}
    \end{flushright}
    \end{minipage}
    \end{minipage}
    \begin{minipage}{\textwidth}
    \vspace{1.4ex}
    \begin{minipage}[t]{0.16\textwidth}
    \@vorlesungsnummer
    \end{minipage}
    \hfill
    \begin{minipage}[t]{0.84\textwidth}
    \textbf{\Large \@vorlesungstitel} \ \if\@semester\else(\@semester)\fi
    \hfill \textbf{\Large Musterl\"{o}sung\hspace{1pt}}
    \end{minipage}\\[0.2cm]
    %\rule[0.3cm]{\textwidth}{.5pt}
    \begin{minipage}[t]{0.16\textwidth}
    \textbf{\"{U}bung \@uebungsnummer} 
    \end{minipage}
    \hfill 
    \begin{minipage}[t]{0.84\textwidth}
    \textbf{\@thema}\\
    \end{minipage}\\[-1.6ex]
    \rule{\textwidth}{.5pt}
    \vspace{-1.3cm}
    \begin{flushright}
    {\tiny \if\@info\else{\vspace{0.3cm}\@info\vspace{0.1cm}}\fi}
    \end{flushright}
    \end{minipage}}}
\fi

\ifsheet
\renewcommand{\maketitle}{
    {\fontsize{10}{15}\fontfamily{cmss}\selectfont
    \vspace*{-1.3cm}
    \begin{minipage}[b]{\textwidth}
    \begin{minipage}[t]{0.4\textwidth}
    \hspace{-0.33cm}
    \includegraphics[scale=0.8]{img/ethlogo}
    \end{minipage}
    \hfill
    \begin{minipage}[t]{0.6\textwidth}
    \vspace*{-1.3cm}
    \begin{flushright}
    \vspace{-0.3cm}
    \includegraphics[scale=1.2]{img/mcllogo}\\
    \vspace{-0.1cm}
    {\fontsize{6}{8}\fontshape{sl}\selectfont
    Prof. R. D'Andrea\hspace*{2.2pt}\linebreak 
    Prof. L. Guzzella\hspace*{2.2pt}\par}
    \end{flushright}
    \end{minipage}
    \end{minipage}
    \begin{minipage}{\textwidth}
    \vspace{1.4ex}
    \begin{minipage}[t]{0.16\textwidth}
    \@vorlesungsnummer
    \end{minipage}
    \hfill
    \begin{minipage}[t]{0.84\textwidth}
    \textbf{\Large \@vorlesungstitel} \ \if\@semester\else(\@semester)\fi
    \end{minipage}\\[0.2cm]
    %\rule[0.3cm]{\textwidth}{.5pt}
    %\begin{minipage}[t]{0.16\textwidth}
    %\textbf{Information Sheet} 
    %\end{minipage}\\[-1.6ex]
    \rule{\textwidth}{.5pt}
    %\vspace{-1.3cm}
    %\begin{flushright}
    %{\tiny \if\@info\else{\vspace{0.3cm}\@info\vspace{0.1cm}}\fi}
    %\end{flushright}
    \end{minipage}}}
\fi

\ifuebung
\else
    \AtEndDocument{\vfill{\footnotesize Alle Angaben ohne Gew\"{a}hr.}}
\fi     


%%%%%%%
% Revision history
% 
% 2003/10/27 new pagestyle definitions for option 'loesung'
% 2003/10/27 modified command \zweis
% 2003/10/27 new 'aufzL' environment
% 2004/03/17 bugfix (long thema-descriptions in 'loesung' environment)
% 2004/05/13 modified tteilaufg environment (labelsep, labelwidth)
% 2005/11/15 reduced number of required packages, new option a4paper
% 2005/11/15 new options noname, nodate, and novisum
% 2005/11/15 new enumL environment
% 2006/11/20 caption style added
% 2007/02/28 Hint that no responsibility is taken for the correctness 
%            of the information
%
% 2007/10/16 Changes made by Moritz Oetiker, : Replaced Prof. Geering by Prof. D'Andrea,
%            re-activated previously uncommented Package Inclusions.
%
